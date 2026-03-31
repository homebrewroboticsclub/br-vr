#!/usr/bin/env python3
"""Serve teleop_fetch/web (dataset dashboard, debug pages) via HTTP."""

import os
import threading

import rospy
import rospkg
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer


def main():
    rospy.init_node("dataset_web_server", anonymous=False)
    port = int(rospy.get_param("~port", 3002))
    host = rospy.get_param("~host", "0.0.0.0")

    try:
        pkg_path = rospkg.RosPack().get_path("teleop_fetch")
    except rospkg.ResourceNotFound as e:
        rospy.logerr("dataset_web_server: package teleop_fetch not found: %s", e)
        return

    web_root = os.path.join(pkg_path, "web")
    if not os.path.isdir(web_root):
        rospy.logerr("dataset_web_server: missing web directory: %s", web_root)
        return

    os.chdir(web_root)
    try:
        httpd = ThreadingHTTPServer((host, port), SimpleHTTPRequestHandler)
    except OSError as e:
        rospy.logerr(
            "dataset_web_server: cannot bind %s:%s (%s). "
            "Another process may use the port or a second roslaunch shares this rosmaster.",
            host,
            port,
            e,
        )
        raise

    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    rospy.loginfo(
        "dataset_web_server: http://%s:%s/ (root %s)",
        host if host != "0.0.0.0" else "<robot-ip>",
        port,
        web_root,
    )

    def _shutdown():
        httpd.shutdown()

    rospy.on_shutdown(_shutdown)
    rospy.spin()
    httpd.server_close()


if __name__ == "__main__":
    main()
