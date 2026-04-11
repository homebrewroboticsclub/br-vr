# DATA NODE: `peaqClaim` in dataset upload from robot

**Audience:** developers of the DATA_NODE service that receives robot datasets (`POST /sessions/upload` multipart).

**Robot-side:** [dataset_upload_server.py](../scripts/dataset_upload_server.py) adds an optional multipart part when `metadata.json` contains `peaqClaim`.

---

## 1. Multipart upload (`POST /sessions/upload`)

Existing parts (unchanged): `datasetId`, `taskName`, `label`, optional `operatorSessionMeta` (JSON), `file` (`.hbr.tar.gz`).

**New optional part:**

| Name | Content-Type | Description |
|------|----------------|-------------|
| `peaqClaim` | `application/json` | JSON object: same structure as RAID `peaq_claim` / robot `metadata.peaqClaim` (see [RAID_APP_PEAQ_CLAIM_SPEC.md](RAID_APP_PEAQ_CLAIM_SPEC.md) §3). |

If the part is **absent**, behavior is unchanged (backward compatible).

---

## 2. Processing expectations

1. Parse multipart; if `peaqClaim` is present, validate JSON and store with the session/dataset record (DB column, JSONB, or sidecar metadata). If the robot never received a claim from RAID (Peaq issuance blocked, `claim_not_ready`, etc.), the part may be absent — same as today’s robot behavior; see [PEAQ_RAID_CLAIM.md](https://github.com/deushon/rospy_x402/blob/DEV/DOC/PEAQ_RAID_CLAIM.md) § Operational status (`rospy_x402`, branch `DEV`).
2. Index optional fields for search: e.g. `help_request_id`, `network`, `document.id` (if DID-shaped).
3. Do **not** require `peaqClaim` for acceptance of the upload.

---

## 3. Fallback JSON session create

If the robot uses the alternate path `POST /sessions` with JSON body (`datasetId`, `taskName`, `label`, `sourcePath`), optionally accept:

```json
{
  "datasetId": "...",
  "taskName": "...",
  "label": "...",
  "sourcePath": "...",
  "peaqClaim": { }
}
```

Omit `peaqClaim` if unknown.

---

## 4. Example snippet (multipart)

```
Content-Disposition: form-data; name="peaqClaim"
Content-Type: application/json

{"schema_version":1,"network":"peaq-agung","help_request_id":"…","document":{}}
```
