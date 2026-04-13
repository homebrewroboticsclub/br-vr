# DATA_NODE — teleop recovery metadata and training tiers

**Audience:** DATA_NODE service and ML pipeline developers.  
**Robot writers:** `br-vr-dev-sinc` (`metadata.json` inside `.hbr`, multipart `operatorSessionMeta` on `POST /sessions/upload`).  
**Version:** 2026-04-11 — **backward compatible:** all new keys are **optional**. Absence MUST preserve legacy behavior.

## 1. Authority

- **Canonical:** `metadata.json` inside the uploaded `.tar.gz` / `.hbr` archive.
- **Convenience copy:** multipart part `operatorSessionMeta` (JSON) on `POST /sessions/upload` — if keys disagree, **archive wins** (same rule as [`DATA_NODE_OPERATOR_SESSION_SPEC.md`](DATA_NODE_OPERATOR_SESSION_SPEC.md)).

## 2. New optional fields (archive + duplicate in `operatorSessionMeta`)

| Field | Type | Values | Meaning |
|-------|------|--------|---------|
| `datasetTrainingTier` | string | `full`, `recovery_slice`, `robot_only` | Hint for training inclusion. |
| `operatorDataCompleteness` | string | `complete`, `partial`, `absent` | Coverage of operator trajectory in this episode. |
| `sessionOutcome` | object | see §2.1 | Link to KYR session end and robot-side outcome. |

Parsers MUST ignore unknown keys and tolerate missing fields.

### 2.1 `sessionOutcome` object (optional)

| Key | Type | Description |
|-----|------|-------------|
| `kyrSessionId` | string | KYR session id if known. |
| `closureReason` | string | Same string as KYR `close_session` / receipt `closure_reason` when available. |
| `endedBy` | string | `robot_watchdog` \| `operator_service` \| `ly_button` \| `vr_lifecycle` \| `unknown` |
| `utcIso` | string | When outcome was finalized (ISO-8601 UTC). |

## 3. Semantics for training (non-normative recommendations)

- **`full`:** default tier when operator data is complete and session ended normally.
- **`recovery_slice`:** robot-side partial episode (e.g. watchdog or disconnect) but **sufficient robot frames** for offline use; DATA_NODE may still apply QC (minimum duration, minimum robot frame count).
- **`robot_only`:** no usable operator trajectory (`operatorDataCompleteness` = `absent`); may still be stored for state-only models.

**GR00T / policy:** filter in DATA_NODE or dataloader config (e.g. exclude `robot_only` or require `recovery_slice` + `robot_frame_count >= N`). **N is a DATA_NODE parameter**, not enforced by the robot.

## 4. Legacy archives (no new fields)

If `datasetTrainingTier` is absent:

- Infer a **display-only** tier from existing signals: presence/size of `operator/operator_state.bin`, `durationSec`, `uploadStatus`.
- Do **not** fail ingest.

## 5. Changes required on your side (DATA_NODE)

| Item | Required? |
|------|-----------|
| Accept and store new optional JSON fields | **Optional** — if ignored, JSON still lands in blob storage if you persist raw metadata. |
| Index `datasetTrainingTier` / `operatorDataCompleteness` for UI or SQL | **Recommended** for operator dashboards and training selection. |
| Training pipeline filters for GR00T | **Recommended** — product-specific. |

**Mandatory server change:** **none** for backward compatibility; existing `POST /sessions/upload` clients remain valid.
