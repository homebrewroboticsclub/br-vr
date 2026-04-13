#!/usr/bin/env python3
"""
teleop_fetch - unified VR teleoperation node.
Single point of publication to bus_servo.
"""

import json
from datetime import datetime, timezone

import rospy
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from ainex_interfaces.msg import HeadState
from ros_robot_controller.msg import SetBusServosPosition

# Dynamic imports to handle potential missing message types before they are built
try:
    from teleop_fetch.srv import ReceiveGrant, ReceiveGrantResponse, EndSession, EndSessionResponse
except ImportError:
    pass
try:
    from rospy_x402.srv import CompleteTeleopPayment
except ImportError:
    CompleteTeleopPayment = None  # type: ignore
try:
    from KYR.srv import OpenSession, CloseSession
except ImportError:
    pass

from teleop_fetch.config import load_config
from teleop_fetch.vr_adapter import (
    VRData,
    pose_array_to_vr_data,
    joint_state_to_dict,
    update_vr_data_from_joints,
)
from teleop_fetch.head_controller import (
    compute_head_targets,
    create_head_state_msg,
)
from teleop_fetch.start_stop_controller import (
    build_arm_start_positions_msg,
    build_reset_grippers_msg,
)
from teleop_fetch.operator_buttons import rising_edge

# Quest left controller: L_X = start streaming, L_Y = stop (joints on ~vr_input/joints_topic)
_BUTTON_THRESH = 0.5

# Global param for DATA_NODE correlation (see DOC/DATA_NODE_INGEST_AND_EVENTS_SPEC.md).
_KYR_SESSION_CORR_PARAM = "/teleop_fetch/current_kyr_session_id"


class TeleopNode:
    def __init__(self):
        rospy.init_node('teleop_fetch', anonymous=False)
        self.config = load_config()

        # State machine: IDLE, REQUESTED, PENDING_GRANT, ACTIVE, FINISHED, FAILED
        self.session_state = 'IDLE'
        self.current_session_id = None

        # VR data cache
        self.vr_data = VRData()

        # After KYR ACTIVE: head free; arm stream to KYR gated (see ~arm_stream_requires_lx); L_Y disarms.
        self.arm_stream_requires_lx = bool(self.config['arm_stream_requires_lx'])
        self._joint_lx_name = self.config['joint_name_lx']
        self._joint_ly_name = self.config['joint_name_ly']
        self.end_session_on_second_ly = bool(self.config['end_session_on_second_ly'])
        self._ly_disarmed_stream_once = False
        self.operator_armed = False
        self._prev_lx = 0.0
        self._prev_ly = 0.0
        self._xy_edges_need_sync = True

        self._direct_teleop = not bool(self.config['teleop_require_kyr_session'])

        # Operator presence watchdog (KYR ACTIVE only)
        self._operator_presence_last_msg_time = None  # rospy.Time
        self._watchdog_suspended = False
        self._pause_deadline_rostime = None  # rospy.Time or None
        self._watchdog_teardown_scheduled = False
        self._session_heartbeat_prev_log = rospy.Time(0)
        op_timeout = float(self.config.get('operator_presence_timeout_sec', 15.0))
        self._operator_presence_timeout = rospy.Duration(op_timeout)
        self._pause_max_sec = float(self.config.get('operator_presence_pause_max_sec', 600.0))
        self._operator_presence_watchdog_enabled = bool(
            self.config.get('operator_presence_watchdog_enabled', True)
        )
        check_hz = max(0.2, float(self.config.get('operator_presence_check_hz', 1.0)))
        self._watchdog_timer = None
        if not self._direct_teleop and self._operator_presence_watchdog_enabled:
            self._watchdog_timer = rospy.Timer(
                rospy.Duration(1.0 / check_hz),
                self._operator_presence_timer_cb,
            )
        self._kyr_gateway = bool(self.config['use_kyr_servo_gateway'])
        if self._direct_teleop and self._kyr_gateway:
            rospy.logwarn(
                'teleop_fetch: teleop_require_kyr_session=false with use_kyr_servo_gateway=true — '
                'KYR proxy will drop servo traffic until open_session; use use_kyr_servo_gateway:=false on bench.'
            )

        servo_out = (
            '/kyr/bus_servo_in' if self._kyr_gateway else str(self.config['servo_topic'])
        )
        self.servo_pub = rospy.Publisher(
            servo_out,
            SetBusServosPosition,
            queue_size=1,
        )
        self.head_pan_pub = rospy.Publisher(
            self.config['head']['pan_topic'],
            HeadState,
            queue_size=1,
        )
        self.head_tilt_pub = rospy.Publisher(
            self.config['head']['tilt_topic'],
            HeadState,
            queue_size=1,
        )
        # Latched: operator logs default to stop_control until L_X arms arm streaming (get_control).
        self.teleop_state_pub = rospy.Publisher(
            self.config['teleop_state_topic'],
            String,
            queue_size=1,
            latch=True,
        )
        if self._direct_teleop:
            self.session_state = 'ACTIVE'
            self.operator_armed = not self.arm_stream_requires_lx
            self._xy_edges_need_sync = True
            self._ly_disarmed_stream_once = False
            if self.operator_armed:
                self._publish_teleop_state('get_control')
            else:
                self._publish_teleop_state('stop_control')
            self._publish_arm_start_position()
            rospy.loginfo(
                'teleop_fetch: direct / bench mode (teleop_require_kyr_session=false), servo_out=%s',
                servo_out,
            )
        else:
            self._publish_teleop_state('stop_control')

        self._clear_kyr_session_correlation_param()

        # Subscribers
        rospy.Subscriber(
            self.config['poses_topic'],
            PoseArray,
            self._pose_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.config['joints_topic'],
            JointState,
            self._joints_callback,
            queue_size=10,
        )
        rospy.Subscriber(
            self.config['arm_servo_targets_topic'],
            SetBusServosPosition,
            self._arm_targets_callback,
            queue_size=10,
        )
        _life_topic = (self.config.get('operator_presence_lifecycle_topic') or '').strip()
        if not self._direct_teleop and _life_topic:
            rospy.Subscriber(_life_topic, String, self._lifecycle_callback, queue_size=10)

        # Services for lifecycle
        try:
            if not self._direct_teleop:
                rospy.Service('~receive_grant', ReceiveGrant, self._handle_receive_grant)
            rospy.Service('~end_session', EndSession, self._handle_end_session)
        except NameError:
            rospy.logwarn("teleop_fetch services not available. Run catkin_make and source first.")

        # KYR clients (unused in direct bench mode)
        if not self._direct_teleop:
            self.kyr_open_session = rospy.ServiceProxy('/kyr/open_session', OpenSession)
            self.kyr_close_session = rospy.ServiceProxy('/kyr/close_session', CloseSession)
        else:
            self.kyr_open_session = None
            self.kyr_close_session = None

        rospy.loginfo(
            'teleop_fetch initialized: session_state=%s, kyr_gateway=%s, arm_stream_requires_lx=%s, '
            'arm_buttons %s/%s on %s',
            self.session_state,
            self._kyr_gateway,
            self.arm_stream_requires_lx,
            self._joint_lx_name,
            self._joint_ly_name,
            self.config['joints_topic'],
        )
        if self._watchdog_timer:
            rospy.loginfo(
                'teleop_fetch: operator presence watchdog enabled (timeout=%.1fs, lifecycle_topic=%r)',
                self._operator_presence_timeout.to_sec(),
                _life_topic or '(disabled)',
            )
        if CompleteTeleopPayment is None:
            rospy.logwarn(
                'teleop_fetch: CompleteTeleopPayment srv not importable — SOL payment after sessions disabled. '
                'Rebuild catkin with rospy_x402 + teleop_fetch.'
            )

    @staticmethod
    def _clear_kyr_session_correlation_param():
        try:
            rospy.delete_param(_KYR_SESSION_CORR_PARAM)
        except KeyError:
            pass

    @staticmethod
    def _set_kyr_session_correlation_param(session_id: str):
        rospy.set_param(_KYR_SESSION_CORR_PARAM, session_id)

    def _complete_teleop_payment_optional(self, receipt_payload):
        """SOL transfer to operator via rospy_x402 (same wallet as x402_buy_service)."""
        if CompleteTeleopPayment is None:
            rospy.logwarn('complete_teleop_payment skipped: rospy_x402.srv not available')
            return
        try:
            rospy.wait_for_service('/x402/complete_teleop_payment', timeout=5.0)
            proxy = rospy.ServiceProxy('/x402/complete_teleop_payment', CompleteTeleopPayment)
            out = proxy(receipt_payload)
            if out.success:
                sig = (out.payment_signature or '').strip()
                if not sig:
                    rospy.logwarn(
                        'complete_teleop_payment: success but NO on-chain transfer — %s '
                        '(common cause: grant/receipt still has operator_pubkey pending_from_raid until RAID sends real address)',
                        out.message,
                    )
                elif sig == 'skipped_zero_amount':
                    rospy.logwarn(
                        'complete_teleop_payment: zero amount, no transfer — %s',
                        out.message,
                    )
                else:
                    rospy.loginfo(
                        'complete_teleop_payment: %s payment_signature=%s',
                        out.message,
                        sig,
                    )
            else:
                rospy.logwarn('complete_teleop_payment failed: %s', out.message)
        except rospy.ROSException as e:
            rospy.logwarn('complete_teleop_payment unavailable (no payment): %s', e)

    def _handle_receive_grant(self, req):
        if self._direct_teleop:
            return ReceiveGrantResponse(
                success=False,
                message="teleop_require_kyr_session is false (direct / bench mode)",
            )
        if self.session_state in ['ACTIVE', 'PENDING_GRANT']:
            return ReceiveGrantResponse(success=False, message="Session already active or pending")
        
        self.session_state = 'PENDING_GRANT'
        rospy.loginfo("Received grant, opening session in KYR...")

        try:
            rospy.wait_for_service('/kyr/open_session', timeout=2.0)
            res = self.kyr_open_session(req.grant_payload, req.signature)
            if res.success:
                self._clear_session_outcome_params()
                self.session_state = 'ACTIVE'
                self.current_session_id = res.session_id
                self._set_kyr_session_correlation_param(res.session_id)
                rospy.loginfo(f"KYR session {res.session_id} opened. State -> ACTIVE")
                self._watchdog_teardown_scheduled = False
                self._watchdog_suspended = False
                self._pause_deadline_rostime = None
                self._operator_presence_last_msg_time = rospy.Time.now()
                self._xy_edges_need_sync = True
                self._ly_disarmed_stream_once = False
                self._publish_arm_start_position()
                self._publish_teleop_state('stop_control')
                if self.arm_stream_requires_lx:
                    self.operator_armed = False
                    rospy.loginfo(
                        'Session ACTIVE: arm stream waits for rising edge on joint "%s" (%s), or set ~arm_stream_requires_lx:=false',
                        self._joint_lx_name,
                        self.config['joints_topic'],
                    )
                else:
                    self.operator_armed = True
                    self._publish_teleop_state('get_control')
                    rospy.loginfo(
                        'Session ACTIVE: arm_stream_requires_lx=false — arm targets forwarded immediately (/teleop_state: get_control)'
                    )
                return ReceiveGrantResponse(success=True, message=res.message)
            else:
                self.session_state = 'FAILED'
                self._clear_kyr_session_correlation_param()
                rospy.logwarn(f"KYR denied session: {res.message}. State -> FAILED")
                return ReceiveGrantResponse(success=False, message=res.message)
        except rospy.ServiceException as e:
            self.session_state = 'FAILED'
            self._clear_kyr_session_correlation_param()
            msg = f"Failed to call KYR open_session: {e}"
            rospy.logerr(msg)
            return ReceiveGrantResponse(success=False, message=msg)

    @staticmethod
    def _clear_session_outcome_params():
        """Clear dataset correlation snapshot from the previous KYR session."""
        for param in (
            "/teleop_fetch/last_closed_kyr_session_id",
            "/teleop_fetch/last_session_closure_reason",
            "/teleop_fetch/last_session_ended_by",
            "/teleop_fetch/last_session_outcome_utc_iso",
        ):
            try:
                rospy.delete_param(param)
            except KeyError:
                pass

    def _persist_session_outcome_for_dataset(self, session_id: str, reason: str, ended_by: str):
        """Ros params read by teleop_fetch HBR merge (see DOC/DATA_NODE_TELEOP_RECOVERY_INGEST_SPEC.md)."""
        try:
            rospy.set_param("/teleop_fetch/last_closed_kyr_session_id", session_id or "")
            rospy.set_param("/teleop_fetch/last_session_closure_reason", reason or "")
            rospy.set_param("/teleop_fetch/last_session_ended_by", ended_by or "unknown")
            rospy.set_param(
                "/teleop_fetch/last_session_outcome_utc_iso",
                datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
            )
        except Exception:
            pass

    def _finalize_kyr_session_and_pay(self, reason: str, ended_by: str = "operator_service"):
        """
        ACTIVE → stop arm UI, KYR close_session, optional SOL to operator.
        Used by /teleop_fetch/end_session, second L_Y, watchdog, and VR lifecycle.
        """
        if self._direct_teleop:
            return False, "No KYR session in direct teleop mode"
        if self.session_state != 'ACTIVE':
            return False, "No active session to end"

        closed_id = self.current_session_id
        self._stop_arm_control()

        try:
            rospy.wait_for_service('/kyr/close_session', timeout=5.0)
            res = self.kyr_close_session(closed_id, reason)
            if res.success:
                self._clear_kyr_session_correlation_param()
                self._persist_session_outcome_for_dataset(closed_id or "", reason, ended_by)
                self._complete_teleop_payment_optional(res.receipt_payload)
            return res.success, res.message
        except rospy.ServiceException as e:
            msg = f"Failed to call KYR close_session: {e}"
            rospy.logerr(msg)
            return False, msg

    @staticmethod
    def _map_vr_disconnect_reason(vr_reason: str) -> str:
        r = (vr_reason or "").lower().replace("-", "_")
        mapping = {
            "user_exit": "operator_disconnect_app_exit",
            "app_background": "operator_disconnect_app_exit",
            "ping_exceeded": "operator_disconnect_ping",
            "low_battery": "operator_disconnect_power",
            "network": "operator_disconnect_network",
        }
        return mapping.get(r, "operator_disconnect_unknown")

    def _touch_operator_presence(self):
        if self.session_state == "ACTIVE" or self._direct_teleop:
            self._operator_presence_last_msg_time = rospy.Time.now()

    def _operator_presence_timer_cb(self, _event=None):
        if self._direct_teleop or not self._operator_presence_watchdog_enabled:
            return
        if self.session_state != "ACTIVE":
            return
        now = rospy.Time.now()
        if (now - self._session_heartbeat_prev_log).to_sec() >= 2.0:
            self._session_heartbeat_prev_log = now
            sid = (self.current_session_id or "")[:8]
            rospy.loginfo(
                "teleop_fetch: session ACTIVE kyr_session=%s… armed=%s watchdog_suspended=%s",
                sid,
                self.operator_armed,
                self._watchdog_suspended,
            )
        if self._watchdog_teardown_scheduled:
            return
        if self._watchdog_suspended:
            if self._pause_max_sec > 0 and self._pause_deadline_rostime is not None:
                if now >= self._pause_deadline_rostime:
                    rospy.logwarn(
                        "teleop_fetch: VR pause exceeded operator_presence/pause_max_sec (%.1f); "
                        "resuming watchdog",
                        self._pause_max_sec,
                    )
                    self._watchdog_suspended = False
                    self._pause_deadline_rostime = None
                    self._touch_operator_presence()
            return
        if self._operator_presence_last_msg_time is None:
            return
        if (now - self._operator_presence_last_msg_time) > self._operator_presence_timeout:
            self._watchdog_teardown_scheduled = True
            rospy.logwarn(
                "teleop_fetch: operator presence timeout (%.1fs); closing KYR session",
                self._operator_presence_timeout.to_sec(),
            )
            ok, msg = self._finalize_kyr_session_and_pay(
                "robot_watchdog_timeout",
                ended_by="robot_watchdog",
            )
            if not ok:
                rospy.logwarn("teleop_fetch: watchdog close_session path failed: %s", msg)

    def _lifecycle_callback(self, msg):
        if self._direct_teleop or self.session_state != "ACTIVE":
            return
        try:
            data = json.loads((msg.data or "").strip() or "{}")
        except json.JSONDecodeError:
            rospy.logwarn_throttle(10.0, "teleop_fetch: teleop_lifecycle invalid JSON (throttled)")
            return
        event = str(data.get("event", "")).strip().lower()
        if event == "pause":
            self._watchdog_suspended = True
            if self._pause_max_sec > 0:
                self._pause_deadline_rostime = rospy.Time.now() + rospy.Duration(self._pause_max_sec)
            else:
                self._pause_deadline_rostime = None
            rospy.loginfo("teleop_fetch: VR lifecycle pause (watchdog suspended)")
        elif event == "resume":
            self._watchdog_suspended = False
            self._pause_deadline_rostime = None
            self._touch_operator_presence()
            rospy.loginfo("teleop_fetch: VR lifecycle resume")
        elif event == "disconnect":
            if self._watchdog_teardown_scheduled:
                return
            self._watchdog_teardown_scheduled = True
            mapped = self._map_vr_disconnect_reason(str(data.get("reason", "")).strip())
            ok, msg = self._finalize_kyr_session_and_pay(mapped, ended_by="vr_lifecycle")
            if ok:
                rospy.loginfo("teleop_fetch: VR lifecycle disconnect -> session closed: %s", msg)
            else:
                rospy.logwarn("teleop_fetch: VR lifecycle disconnect finalize failed: %s", msg)

    def _handle_end_session(self, req):
        reason = (req.reason or "").strip() or "end_session_service"
        if self._direct_teleop:
            self.operator_armed = False
            self._xy_edges_need_sync = True
            self._ly_disarmed_stream_once = False
            self._publish_teleop_state('stop_control')
            self._publish_arm_start_position()
            self._reset_head_to_base()
            self._reset_grippers()
            return EndSessionResponse(
                success=True,
                message="direct mode: arm stream stopped; KYR not used (%s)" % reason,
            )
        ok, msg = self._finalize_kyr_session_and_pay(reason, ended_by="operator_service")
        return EndSessionResponse(success=ok, message=msg)

    def _pose_callback(self, msg):
        data = pose_array_to_vr_data(msg)
        self.vr_data.head_pose = data.head_pose
        self.vr_data.head_orientation = data.head_orientation
        self.vr_data.left_hand_pose = data.left_hand_pose
        self.vr_data.right_hand_pose = data.right_hand_pose

        if self.session_state == 'ACTIVE' or self._direct_teleop:
            self._touch_operator_presence()
            self._process_head_control()
            self._process_operator_xy_buttons(self.vr_data.left_x, self.vr_data.left_y)

    def _joints_callback(self, msg):
        joint_dict = joint_state_to_dict(msg)
        update_vr_data_from_joints(self.vr_data, joint_dict)
        lx = float(joint_dict.get(self._joint_lx_name, 0.0))
        ly = float(joint_dict.get(self._joint_ly_name, 0.0))
        if self.session_state == 'ACTIVE' or self._direct_teleop:
            self._touch_operator_presence()
        self._process_operator_xy_buttons(lx, ly)

    def _process_operator_xy_buttons(self, lx, ly):
        if self.session_state != 'ACTIVE' and not self._direct_teleop:
            return
        if self._xy_edges_need_sync:
            self._prev_lx = lx
            self._prev_ly = ly
            self._xy_edges_need_sync = False
            return
        y_edge = rising_edge(self._prev_ly, ly, _BUTTON_THRESH)
        x_edge = rising_edge(self._prev_lx, lx, _BUTTON_THRESH)
        self._prev_lx = lx
        self._prev_ly = ly
        if y_edge and self.operator_armed:
            self._operator_y_disarm()
        elif (
            y_edge
            and not self.operator_armed
            and self.end_session_on_second_ly
            and self._ly_disarmed_stream_once
        ):
            if self._direct_teleop:
                rospy.loginfo("Second L_Y: ignored in direct teleop mode (no KYR session)")
            else:
                ok, msg = self._finalize_kyr_session_and_pay(
                    "operator_second_ly_press",
                    ended_by="ly_button",
                )
                if ok:
                    rospy.loginfo("Second L_Y: KYR session closed and billing path run: %s", msg)
                else:
                    rospy.logwarn("Second L_Y: failed to finalize session: %s", msg)
        elif x_edge and not self.operator_armed:
            self.operator_armed = True
            self._publish_teleop_state('get_control')
            rospy.loginfo(
                'Arm stream armed: joint "%s" (/teleop_state: get_control)',
                self._joint_lx_name,
            )

    def _process_head_control(self):
        pan, tilt = compute_head_targets(
            self.vr_data.head_orientation,
            self.config['head'],
        )
        if pan is not None and tilt is not None:
            duration = self.config['head']['movement_duration']
            pan_msg = create_head_state_msg(pan, duration)
            tilt_msg = create_head_state_msg(tilt, duration)
            self.head_pan_pub.publish(pan_msg)
            self.head_tilt_pub.publish(tilt_msg)

    def _stop_arm_control(self):
        self.operator_armed = False
        self._xy_edges_need_sync = True
        self._publish_teleop_state('stop_control')
        self.session_state = 'FINISHED'
        rospy.loginfo('teleop_fetch: KYR session ended — arm control disabled (FINISHED)')
        self._publish_arm_start_position()
        self._reset_head_to_base()
        self._reset_grippers()

    def _operator_y_disarm(self):
        """L_Y: stop streaming; KYR session stays ACTIVE until end_session or second L_Y."""
        self.operator_armed = False
        self._ly_disarmed_stream_once = True
        self._xy_edges_need_sync = True
        self._publish_teleop_state('stop_control')
        self._publish_arm_start_position()
        self._reset_head_to_base()
        self._reset_grippers()
        rospy.loginfo('L_Y: operator disarmed (session still ACTIVE)')

    def _publish_teleop_state(self, data):
        """Latched operator log: boot+ACTIVE→stop_control; L_X→get_control; L_Y if armed or end_session→stop_control."""
        self.teleop_state_pub.publish(String(data=data))
        rospy.logdebug('Published /teleop_state: %s', data)

    def _publish_arm_start_position(self):
        msg = build_arm_start_positions_msg(self.config, duration=0.1)
        self.servo_pub.publish(msg)
        rospy.logdebug('Published arm start positions')

    def _reset_head_to_base(self):
        pan_msg = create_head_state_msg(0.0, self.config['head']['movement_duration'])
        tilt_msg = create_head_state_msg(0.0, self.config['head']['movement_duration'])
        self.head_pan_pub.publish(pan_msg)
        self.head_tilt_pub.publish(tilt_msg)
        rospy.logdebug('Head reset to base')

    def _reset_grippers(self):
        msg = build_reset_grippers_msg(self.config)
        self.servo_pub.publish(msg)
        rospy.logdebug('Grippers reset')

    def _arm_targets_callback(self, msg):
        """Forward arm targets from fast_ik to KYR proxy (or direct servo topic) when session armed."""
        active_ok = self.session_state == 'ACTIVE' or self._direct_teleop
        if not active_ok or not self.operator_armed:
            if (
                active_ok
                and not self.operator_armed
                and self.arm_stream_requires_lx
                and msg.position
            ):
                rospy.logwarn_throttle(
                    10.0,
                    'teleop_fetch: dropping arm_servo_targets (ACTIVE but not armed). '
                    'Press joint "%s" on %s or set param ~arm_stream_requires_lx:=false.',
                    self._joint_lx_name,
                    self.config['joints_topic'],
                )
            return
        self.servo_pub.publish(msg)

    def run(self):
        rospy.spin()


def main():
    try:
        node = TeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo('teleop_fetch stopped')


if __name__ == '__main__':
    main()
