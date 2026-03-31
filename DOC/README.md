# VR Teleop — documentation index

All technical documentation lives in **`DOC/`**. The root [README.md](../README.md) is the human-facing overview and setup entry point.

## Stack architecture

- [ARCHITECTURE.md](ARCHITECTURE.md) — abstraction layers, topics, flow Quest → remapper → IK → `teleop_fetch`.
- **Optional x402 / RAID:** when `rospy_x402` is installed, `teleop_node` can call `/x402/complete_teleop_payment` after a session. Full-cycle RAID contract (reference implementation): [RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md](https://github.com/deushon/rospy_x402/blob/DEV/DOC/RAID_APP_TELEOP_HELP_FULL_CYCLE_X402_SPEC.md) (`rospy_x402`, branch `DEV`).

## Project state and tasks

- [PROJECT_STATE.md](PROJECT_STATE.md) — package and component status.
- [TODO.md](TODO.md) — known issues and backlog.

## Contributing

Use **English** in source code, comments, commit messages, and everything under **`DOC/`** so the package stays suitable for a public repository.

### Pull request when GitHub reports unrelated histories

If the compare view says there is *nothing to compare* or that the branches have *entirely different commit histories*, your fork was never created from (or rebased onto) the upstream repository, so there is no merge-base. Replay your **current tree** as **one commit** on top of `upstream/main`.

**Remotes:** public upstream is [homebrewroboticsclub/br-vr](https://github.com/homebrewroboticsclub/br-vr) (`teleop_fetch` sources). A team fork such as [deushon/br-vr-dev-sinc](https://github.com/deushon/br-vr-dev-sinc) is `origin`; PRs go **from** that fork **into** `homebrewroboticsclub/br-vr:main`.

**Important:** `upstream` must be **`br-vr`**, not `homebrewroboticsclub/br-vr-dev-sinc` (if it exists). A wrong `upstream` URL leaves you replaying onto the wrong graph, so GitHub still shows unrelated histories. Check with `git remote -v` and fix: `git remote set-url upstream https://github.com/homebrewroboticsclub/br-vr.git`.

```bash
# Upstream (original package repo), not your fork URL
git remote add upstream https://github.com/homebrewroboticsclub/br-vr.git
git fetch upstream

git checkout RX_SPR_2
SNAPSHOT=$(git rev-parse HEAD)

git checkout -B RX_SPR_2 upstream/main
git reset --hard "$SNAPSHOT"
git reset --soft upstream/main
git commit -m "teleop_fetch: align fork with upstream main for public PR"

git push --force-with-lease origin RX_SPR_2
```

Then open the PR from **deushon/br-vr-dev-sinc** branch `RX_SPR_2` into **homebrewroboticsclub/br-vr** `main`.

## Datasets and HBR format

- On the robot: with `enable_dataset_recording` in `teleop.launch`, REST **:9191** and **`dataset_web_server`** start → `http://<robot>:3002/dataset_dashboard.html` (see [ARCHITECTURE.md](ARCHITECTURE.md) §6).
- [TELEOP_DATAS.md](TELEOP_DATAS.md) — headset events, upload API contract.
- [HBR.md](HBR.md) — `.hbr` container format, storage.
- [RAID_APP_DATASET_PROXY_SPEC.md](RAID_APP_DATASET_PROXY_SPEC.md) — **RAID App** (`x402_raid_app`): HTTP reverse proxy to dataset API on robot (`:9191`) for operators via JWT.
- [RAID_APP_PEAQ_CLAIM_SPEC.md](RAID_APP_PEAQ_CLAIM_SPEC.md) — **RAID App**: peaq claim on Agung, `teleop/help` extension and `GET …/peaq/claim`.
- [DATA_NODE_OPERATOR_SESSION_SPEC.md](DATA_NODE_OPERATOR_SESSION_SPEC.md) — **DATA_NODE**: extended teleop session fields, `metadata.json`, multipart `operatorSessionMeta` on `POST /sessions/upload`.
- [DATA_NODE_PEAQ_CLAIM_SPEC.md](DATA_NODE_PEAQ_CLAIM_SPEC.md) — **DATA_NODE**: optional multipart part `peaqClaim` when uploading dataset from robot.

---

New functional areas → add a `DOC/*.md` file and a line in this index; update the root [README.md](../README.md) when user-facing behavior changes.
