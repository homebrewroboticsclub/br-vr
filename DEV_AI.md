# DEV_AI — agent context (teleop_fetch / br-vr-dev-sinc)

## Ecosystem entry point

**Launch, KYR proxy topic wiring, RAID env, dataset ports** — metapackage **`br_bringup`**:  
**[../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md)** and **[../br_bringup/README.md](../br_bringup/README.md)**.

## This repository

Catkin package **`teleop_fetch`**: VR remapper, pose source, dataset recorder/upload, `teleop_node` → KYR-gated servo path when used with **`br_bringup`**. Full VR stack launch: [launch/teleop.launch](launch/teleop.launch) (includes `robot`, `my_package` / `fast_ik_node`).

**Language (agents):** use **English** in this repository (code, comments, `DOC/`). In **chat**, answer in **Russian** when the human writes in Russian. Workspace rules: `ros_ws/.cursor/rules/project-context.mdc`. Public push checklist: [../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md](../br_bringup/DOC/PUBLIC_RELEASE_CHECKLIST.md).

## Related repositories (same overlay)

| Area | DEV_AI | DOC / README |
|------|--------|----------------|
| Ecosystem launch | [../br_bringup/DEV_AI.md](../br_bringup/DEV_AI.md) | [../br_bringup/README.md](../br_bringup/README.md) |
| KYR | [../br-kyr/DEV_AI.md](../br-kyr/DEV_AI.md) | [../br-kyr/DOC/README.md](../br-kyr/DOC/README.md) |
| rospy_x402 (RAID / x402) | [../rospy_x402/DEV_AI.md](../rospy_x402/DEV_AI.md) | [../rospy_x402/DOC/README.md](../rospy_x402/DOC/README.md) |
| MoveIt / URDF context | [../robot/DEV_AI.md](../robot/DEV_AI.md) | [../robot/DOC/README.md](../robot/DOC/README.md) |
| `fast_ik_node` | [../my_package/DEV_AI.md](../my_package/DEV_AI.md) | [../my_package/DOC/README.md](../my_package/DOC/README.md) |
| URDF source (`ainex_description`) | — | [../ainex_simulations/DOC/README.md](../ainex_simulations/DOC/README.md) |

**Alternate checkout:** sibling [../teleop_fetch/DEV_AI.md](../teleop_fetch/DEV_AI.md) — keep only one `teleop_fetch` package in the overlay (see `CATKIN_IGNORE` there).

## Layout

- Nodes and configs under `scripts/`, `config/`, `web/`.
- **Documentation index:** [DOC/README.md](DOC/README.md). Human-oriented overview: [README.md](README.md).

## Responsibilities when changing code

1. **Documentation** — update affected `DOC/*.md`; new area → new file + line in [DOC/README.md](DOC/README.md). If topics or launch args change for the full stack, sync **[../br_bringup/README.md](../br_bringup/README.md)** / **[launch/ecosystem.launch](../br_bringup/launch/ecosystem.launch)** when applicable.
2. **Tests** — add tests for new behaviour when the package defines them; run:  
   `cd /home/ubuntu/ros_ws && source devel/setup.bash && catkin_make run_tests --pkg teleop_fetch` (if registered).
3. **Commit** — clear English messages.

## Build and launch (short)

```bash
cd /home/ubuntu/ros_ws/devel && source setup.bash
cd /home/ubuntu/ros_ws && catkin build teleop_fetch
roslaunch teleop_fetch teleop.launch
```

Full stack with KYR + x402: `roslaunch br_bringup ecosystem.launch`.
