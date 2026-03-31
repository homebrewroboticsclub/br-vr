#!/usr/bin/env python3
"""
HTTP server for headset upload_dataset endpoint.
"""

import json
import os
import shutil
import tarfile
import time
import uuid
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from io import BytesIO
from urllib import request as urlrequest
from urllib.error import HTTPError
from urllib.parse import quote
from urllib.parse import parse_qs, urlparse

import rospy

from teleop_fetch.upload_models import UploadPayload


class UploadServerConfig:
    def __init__(self):
        def p(name, default):
            return rospy.get_param("~" + name, default)

        self.host = p("upload_api/host", p("host", "0.0.0.0"))
        self.port = int(p("upload_api/port", p("port", 9191)))
        self.path = p("upload_api/path", p("path", "/upload_dataset"))
        self.output_root = os.path.expanduser(p("output_root", "~/.teleop_fetch_datasets/hbr"))
        self.cache_root = os.path.expanduser(p("cache_root", "~/.teleop_fetch_datasets/cache"))
        self.upload_inbox_dir = os.path.expanduser(p("upload_inbox_dir", "~/.teleop_fetch_datasets/upload_inbox"))
        self.logs_dir = os.path.expanduser(p("logs_dir", "~/.teleop_fetch_datasets/logs"))
        os.makedirs(self.output_root, exist_ok=True)
        os.makedirs(self.upload_inbox_dir, exist_ok=True)
        os.makedirs(self.cache_root, exist_ok=True)
        os.makedirs(self.logs_dir, exist_ok=True)


class UploadHandler(BaseHTTPRequestHandler):
    config: UploadServerConfig = None

    def _append_log(self, event: str, payload):
        row = {
            "timeUnixNs": int(time.time_ns()),
            "event": event,
            "payload": payload,
        }
        try:
            path = os.path.join(self.config.logs_dir, "dataset_upload_server.log")
            with open(path, "a", encoding="utf-8") as f:
                f.write(json.dumps(row, ensure_ascii=True) + "\n")
        except OSError:
            pass

    def _metadata_path(self, dataset_id: str) -> str:
        return os.path.join(self.config.output_root, "%s.hbr" % dataset_id, "metadata.json")

    def _read_metadata(self, dataset_id: str) -> dict:
        path = self._metadata_path(dataset_id)
        if not os.path.exists(path):
            return {}
        try:
            with open(path, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            return {}

    def _write_metadata(self, dataset_id: str, metadata: dict) -> None:
        path = self._metadata_path(dataset_id)
        tmp = path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(metadata, f, indent=2, ensure_ascii=True)
            f.write("\n")
        os.replace(tmp, path)

    def _active_dataset_id_from_state(self) -> str:
        path = os.path.join(self.config.cache_root, "session_state.json")
        if not os.path.exists(path):
            return ""
        try:
            with open(path, "r", encoding="utf-8") as f:
                state = json.load(f)
            active = str(state.get("activeDatasetId") or "")
            if active:
                return active
            known = state.get("knownDatasetIds") or []
            if isinstance(known, list) and len(known) == 1:
                return str(known[0])
        except Exception:
            return ""
        return ""

    def _ensure_record_ids(self, payload: dict, default_record_id: str) -> dict:
        if not isinstance(payload, dict):
            return payload
        records = payload.get("records")
        if not isinstance(records, list):
            return payload
        top_record_id = ""
        for key in ("recordId", "record_id", "datasetId", "dataset_id", "id"):
            if payload.get(key):
                top_record_id = str(payload.get(key))
                break
        generated = []
        for idx, rec in enumerate(records):
            if not isinstance(rec, dict):
                continue
            record_id = ""
            for key in ("recordId", "record_id", "datasetId", "dataset_id", "id"):
                if rec.get(key):
                    record_id = str(rec.get(key))
                    break
            if not record_id and isinstance(rec.get("data"), dict):
                data = rec["data"]
                for key in ("recordId", "record_id", "datasetId", "dataset_id", "id"):
                    if data.get(key):
                        record_id = str(data.get(key))
                        break
            if not record_id:
                record_id = default_record_id or top_record_id
            if not record_id:
                started_ns = ""
                data = rec.get("data")
                if isinstance(data, dict) and data.get("startedLocalUnixTimeNs"):
                    started_ns = str(data.get("startedLocalUnixTimeNs"))
                suffix = started_ns if started_ns else str(int(time.time_ns()))
                record_id = "uploadonly-%s-%d" % (suffix, idx)
                generated.append(record_id)
            rec["recordId"] = record_id
            if isinstance(rec.get("data"), dict):
                rec["data"]["recordId"] = record_id
        if generated:
            self._append_log("upload_record_id_generated", {"recordIds": generated})
        return payload

    def log_message(self, fmt, *args):
        rospy.loginfo("upload_http: " + fmt, *args)

    def do_OPTIONS(self):  # noqa: N802
        self.send_response(HTTPStatus.NO_CONTENT)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):  # noqa: N802
        parsed = urlparse(self.path)
        if parsed.path == "/dataset_status":
            self._reply(HTTPStatus.OK, self._collect_dataset_status())
            return
        if parsed.path == "/dataset_logs":
            query = parse_qs(parsed.query or "")
            lines = int(query.get("lines", ["200"])[0])
            self._reply(HTTPStatus.OK, self._collect_logs(lines=max(1, min(lines, 2000))))
            return
        if parsed.path.startswith("/dataset_download/"):
            dataset_id = parsed.path.replace("/dataset_download/", "", 1).strip()
            self._serve_dataset_archive(dataset_id)
            return
        self._reply(HTTPStatus.NOT_FOUND, {"status": "error", "message": "Unknown endpoint"})

    def do_POST(self):  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") if parsed.path != "/" else parsed.path
        if path == "/dataset_delete":
            self._handle_dataset_delete()
            return
        if path == "/dataset_push":
            self._handle_dataset_push()
            return
        if path == "/dataset_clear_all":
            self._handle_dataset_clear_all()
            return

        upload_path = self.config.path.rstrip("/") if self.config.path != "/" else self.config.path
        if path != upload_path:
            self._reply(HTTPStatus.NOT_FOUND, {"status": "error", "message": "Unknown endpoint"})
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        default_record_id = self._active_dataset_id_from_state()
        parsed_payload = {}
        try:
            parsed_payload = json.loads(raw.decode("utf-8"))
            parsed_payload = self._ensure_record_ids(parsed_payload, default_record_id=default_record_id)
            raw = json.dumps(parsed_payload).encode("utf-8")
        except Exception:
            parsed_payload = {}
        try:
            model = UploadPayload.from_json_bytes(raw, default_record_id=default_record_id)
        except Exception as exc:
            parsed = parsed_payload
            if not parsed:
                try:
                    parsed = json.loads(raw.decode("utf-8"))
                except Exception:
                    parsed = {"_raw": raw.decode("utf-8", errors="replace")}
            self._append_log(
                "upload_rejected",
                {
                    "error": str(exc),
                    "defaultRecordIdFromState": default_record_id,
                    "topLevelKeys": sorted(list(parsed.keys())) if isinstance(parsed, dict) else [],
                    "recordsType": type(parsed.get("records")).__name__ if isinstance(parsed, dict) else "unknown",
                    "recordsCount": len(parsed.get("records", [])) if isinstance(parsed, dict) and isinstance(parsed.get("records"), list) else 0,
                    "payloadPreview": str(parsed)[:5000],
                },
            )
            self._reply(HTTPStatus.BAD_REQUEST, {"status": "error", "message": str(exc)})
            return

        missing = []
        for record_id in model.record_ids():
            hbr_path = os.path.join(self.config.output_root, "%s.hbr" % record_id)
            if not os.path.isdir(hbr_path):
                missing.append(record_id)
        if missing:
            for record_id in missing:
                root = os.path.join(self.config.output_root, "%s.hbr" % record_id)
                os.makedirs(os.path.join(root, "robot"), exist_ok=True)
                os.makedirs(os.path.join(root, "operator"), exist_ok=True)
                os.makedirs(os.path.join(root, "video"), exist_ok=True)
            self._append_log("upload_session_autocreate", {"recordIds": missing})

        payload = model.payload
        name = "upload_%d_%s.json" % (int(time.time_ns()), uuid.uuid4().hex[:8])
        out = os.path.join(self.config.upload_inbox_dir, name)
        tmp = out + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(payload, f)
        os.replace(tmp, out)
        self._append_log("upload_queued", {"file": name, "recordIds": model.record_ids()})
        self._reply(
            HTTPStatus.OK,
            {
                "status": "ok",
                "recordsQueued": len(model.records),
                "message": "Operator data queued for recorder processing.",
            },
        )

    def _handle_dataset_delete(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
            dataset_id = str(payload.get("datasetId", "")).strip()
            if not dataset_id:
                raise ValueError("datasetId is required")
        except Exception as exc:
            self._reply(HTTPStatus.BAD_REQUEST, {"status": "error", "message": str(exc)})
            return
        root = os.path.join(self.config.output_root, "%s.hbr" % dataset_id)
        if not os.path.isdir(root):
            self._reply(HTTPStatus.NOT_FOUND, {"status": "error", "message": "Dataset not found"})
            return
        try:
            shutil.rmtree(root)
            self._append_log("dataset_deleted", {"datasetId": dataset_id})
            self._reply(HTTPStatus.OK, {"status": "ok", "datasetId": dataset_id})
        except Exception as exc:
            self._reply(HTTPStatus.INTERNAL_SERVER_ERROR, {"status": "error", "message": str(exc)})

    def _handle_dataset_push(self) -> None:
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            payload = json.loads(raw.decode("utf-8"))
            dataset_id = str(payload.get("datasetId", "")).strip()
            data_node_url = str(payload.get("dataNodeUrl", "")).strip()
            upload_path = str(payload.get("uploadPath", "/sessions/upload")).strip() or "/sessions/upload"
            if not dataset_id:
                raise ValueError("datasetId is required")
            if not data_node_url:
                data_node_url = str(
                    rospy.get_param("~auto_push/data_node_url", "http://127.0.0.1:8088")
                ).strip()
            if not data_node_url:
                raise ValueError("dataNodeUrl is required (set in UI or auto_push/data_node_url)")
        except Exception as exc:
            self._reply(HTTPStatus.BAD_REQUEST, {"status": "error", "message": str(exc)})
            return

        root = os.path.join(self.config.output_root, "%s.hbr" % dataset_id)
        if not os.path.isdir(root):
            self._reply(HTTPStatus.NOT_FOUND, {"status": "error", "message": "Dataset not found"})
            return
        try:
            url = data_node_url.rstrip("/") + upload_path
            metadata = self._read_metadata(dataset_id)
            code, resp_body, mode = self._push_to_data_node(url, dataset_id, root, metadata, upload_path)
            metadata = self._read_metadata(dataset_id)
            metadata["uploadStatus"] = "uploaded"
            metadata["uploadedAtUnixNs"] = int(time.time_ns())
            metadata["uploadedTargetUrl"] = url
            metadata["uploadedMode"] = mode
            metadata["uploadLastError"] = ""
            self._write_metadata(dataset_id, metadata)
            self._append_log("dataset_push_ok", {"datasetId": dataset_id, "url": url, "code": code, "mode": mode})
            self._reply(
                HTTPStatus.OK,
                {
                    "status": "ok",
                    "datasetId": dataset_id,
                    "targetUrl": url,
                    "targetStatus": code,
                    "targetBody": resp_body[:1000],
                    "mode": mode,
                },
            )
        except Exception as exc:
            metadata = self._read_metadata(dataset_id)
            metadata["uploadStatus"] = "failed"
            metadata["uploadLastError"] = str(exc)
            metadata["uploadLastAttemptUnixNs"] = int(time.time_ns())
            self._write_metadata(dataset_id, metadata)
            self._append_log("dataset_push_failed", {"datasetId": dataset_id, "error": str(exc), "targetUrl": url})
            self._reply(HTTPStatus.BAD_GATEWAY, {"status": "error", "message": str(exc)})

    def _push_to_data_node(self, url: str, dataset_id: str, dataset_root: str, metadata: dict, upload_path: str = ""):
        archive = self._build_archive_bytes(dataset_root, dataset_id)
        archive_mb = len(archive) / (1024 * 1024)
        boundary = "----teleopfetch-%s" % uuid.uuid4().hex
        body = self._multipart_body(boundary, dataset_id, archive, metadata)
        backoff_sec = [1, 2, 4, 8]
        last_exc = None
        for attempt in range(5):
            try:
                req = urlrequest.Request(url=url, method="POST", data=body)
                req.add_header("Content-Type", "multipart/form-data; boundary=%s" % boundary)
                req.add_header("Content-Length", str(len(body)))
                with urlrequest.urlopen(req, timeout=60) as resp:
                    resp_body = resp.read().decode("utf-8", errors="replace")
                    return int(resp.getcode()), resp_body, "multipart"
            except HTTPError as exc:
                code = int(exc.code)
                resp_body = ""
                try:
                    resp_body = exc.fp.read().decode("utf-8", errors="replace")[:500] if exc.fp else ""
                except Exception:
                    pass
                self._append_log("dataset_push_http_error", {
                    "datasetId": dataset_id, "code": code, "attempt": attempt + 1,
                    "archiveMb": round(archive_mb, 2), "responseBody": resp_body,
                })
                if code == 400:
                    raise RuntimeError("DATA_NODE 400 Bad Request: %s" % (resp_body or str(exc)))
                if code >= 500:
                    last_exc = exc
                    if attempt < 4:
                        time.sleep(backoff_sec[min(attempt, 3)])
                        continue
                    raise
                if code in (404, 405, 415, 422):
                    if "/upload" in (upload_path or url):
                        raise RuntimeError(
                            "DATA_NODE returned %s on %s — multipart upload not available. "
                            "Ensure DATA_NODE implements POST /sessions/upload per ROBOT_SERVICE_INTEGRATION.md"
                            % (code, url)
                        )
                    break
                last_exc = exc
                raise
            except (OSError, TimeoutError) as exc:
                last_exc = exc
                self._append_log("dataset_push_retry", {
                    "datasetId": dataset_id, "attempt": attempt + 1,
                    "archiveMb": round(archive_mb, 2), "error": str(exc),
                })
                if attempt < 4:
                    time.sleep(backoff_sec[min(attempt, 3)])
                    continue
                raise last_exc

        # Mode B: POST /sessions with JSON and sourcePath (only when NOT targeting /sessions/upload).
        sessions_url = url.rstrip("/").rsplit("/", 1)[0] if "/upload" in url else url
        robot_ip = self._guess_robot_ip()
        source_path = "http://%s:%d/dataset_download/%s" % (robot_ip, self.config.port, dataset_id)
        body_json = {
            "datasetId": dataset_id,
            "taskName": str(metadata.get("taskName", "")),
            "label": str(metadata.get("label", "")),
            "sourcePath": source_path,
        }
        req = urlrequest.Request(
            url=sessions_url,
            method="POST",
            data=json.dumps(body_json).encode("utf-8"),
            headers={"Content-Type": "application/json"},
        )
        with urlrequest.urlopen(req, timeout=25) as resp:
            resp_body = resp.read().decode("utf-8", errors="replace")
            return int(resp.getcode()), resp_body, "json_sourcePath"

    def _guess_robot_ip(self) -> str:
        try:
            ws = rospy.get_param("/dataset_recorder/source_ws_url", "ws://127.0.0.1:9090")
            parsed = urlparse(ws)
            if parsed.hostname:
                return parsed.hostname
        except Exception:
            pass
        return "127.0.0.1"

    def _handle_dataset_clear_all(self) -> None:
        removed = {"hbr": 0, "inbox": 0, "logs": 0, "cache": 0}
        try:
            for name in os.listdir(self.config.output_root):
                path = os.path.join(self.config.output_root, name)
                if os.path.isdir(path):
                    shutil.rmtree(path)
                    removed["hbr"] += 1
            for name in os.listdir(self.config.upload_inbox_dir):
                path = os.path.join(self.config.upload_inbox_dir, name)
                if os.path.isfile(path):
                    os.remove(path)
                    removed["inbox"] += 1
            for name in os.listdir(self.config.logs_dir):
                path = os.path.join(self.config.logs_dir, name)
                if os.path.isfile(path):
                    os.remove(path)
                    removed["logs"] += 1
            for name in os.listdir(self.config.cache_root):
                path = os.path.join(self.config.cache_root, name)
                if os.path.isfile(path):
                    os.remove(path)
                    removed["cache"] += 1
            self._append_log("dataset_clear_all", {"removed": removed})
            self._reply(HTTPStatus.OK, {"status": "ok", "removed": removed})
        except Exception as exc:
            self._reply(HTTPStatus.INTERNAL_SERVER_ERROR, {"status": "error", "message": str(exc)})

    def _build_archive_bytes(self, root: str, dataset_id: str) -> bytes:
        bio = BytesIO()
        arcname = "%s.hbr" % dataset_id
        with tarfile.open(fileobj=bio, mode="w:gz") as tar:
            tar.add(root, arcname=arcname)
        return bio.getvalue()

    def _multipart_body(self, boundary: str, dataset_id: str, archive: bytes, metadata: dict = None) -> bytes:
        metadata = metadata or {}
        task_name = str(metadata.get("taskName", "")).strip() or "unknown_task"
        label = str(metadata.get("label", "")).strip() or "unlabeled"
        parts = []
        parts.append(("--%s\r\n" % boundary).encode("utf-8"))
        parts.append(b'Content-Disposition: form-data; name="datasetId"\r\n\r\n')
        parts.append(dataset_id.encode("utf-8"))
        parts.append(b"\r\n")
        parts.append(("--%s\r\n" % boundary).encode("utf-8"))
        parts.append(b'Content-Disposition: form-data; name="taskName"\r\n\r\n')
        parts.append(task_name.encode("utf-8"))
        parts.append(b"\r\n")
        parts.append(("--%s\r\n" % boundary).encode("utf-8"))
        parts.append(b'Content-Disposition: form-data; name="label"\r\n\r\n')
        parts.append(label.encode("utf-8"))
        parts.append(b"\r\n")
        session_meta = {}
        for key in ("source", "generatedUtcIso", "acceptedAtUtcIso"):
            val = metadata.get(key)
            if val is not None and str(val).strip() != "":
                session_meta[key] = val
        tc = metadata.get("teleopControl")
        if isinstance(tc, dict):
            session_meta["teleopControl"] = tc
        if session_meta:
            parts.append(("--%s\r\n" % boundary).encode("utf-8"))
            parts.append(
                b'Content-Disposition: form-data; name="operatorSessionMeta"\r\n'
                b"Content-Type: application/json\r\n\r\n"
            )
            parts.append(json.dumps(session_meta, ensure_ascii=True).encode("utf-8"))
            parts.append(b"\r\n")
        peaq_claim = metadata.get("peaqClaim")
        if isinstance(peaq_claim, dict) and peaq_claim:
            parts.append(("--%s\r\n" % boundary).encode("utf-8"))
            parts.append(
                b'Content-Disposition: form-data; name="peaqClaim"\r\n'
                b"Content-Type: application/json\r\n\r\n"
            )
            parts.append(json.dumps(peaq_claim, ensure_ascii=True).encode("utf-8"))
            parts.append(b"\r\n")
        parts.append(("--%s\r\n" % boundary).encode("utf-8"))
        parts.append(
            ('Content-Disposition: form-data; name="file"; filename="%s.hbr.tar.gz"\r\n' % dataset_id).encode("utf-8")
        )
        parts.append(b"Content-Type: application/gzip\r\n\r\n")
        parts.append(archive)
        parts.append(b"\r\n")
        parts.append(("--%s--\r\n" % boundary).encode("utf-8"))
        return b"".join(parts)

    def _serve_dataset_archive(self, dataset_id: str) -> None:
        if not dataset_id:
            self._reply(HTTPStatus.BAD_REQUEST, {"status": "error", "message": "datasetId is required"})
            return
        root = os.path.join(self.config.output_root, "%s.hbr" % dataset_id)
        if not os.path.isdir(root):
            self._reply(HTTPStatus.NOT_FOUND, {"status": "error", "message": "Dataset not found"})
            return
        try:
            blob = self._build_archive_bytes(root, dataset_id)
            self.send_response(HTTPStatus.OK)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Content-Type", "application/gzip")
            self.send_header("Content-Length", str(len(blob)))
            self.send_header("Content-Disposition", 'attachment; filename="%s.hbr.tar.gz"' % quote(dataset_id))
            self.end_headers()
            self.wfile.write(blob)
        except Exception as exc:
            self._reply(HTTPStatus.INTERNAL_SERVER_ERROR, {"status": "error", "message": str(exc)})

    def _collect_dataset_status(self):
        datasets = []
        try:
            names = [n for n in os.listdir(self.config.output_root) if n.endswith(".hbr")]
        except OSError:
            names = []
        for name in sorted(names):
            root = os.path.join(self.config.output_root, name)
            dataset_id = name[:-4]
            metadata_path = os.path.join(root, "metadata.json")
            operator_bin = os.path.join(root, "operator", "operator_state.bin")
            robot_bin = os.path.join(root, "robot", "robot_state.bin")
            upload_complete = os.path.exists(operator_bin) and os.path.getsize(operator_bin) > 0
            robot_ready = os.path.exists(robot_bin) and os.path.getsize(robot_bin) > 0
            metadata = {}
            if os.path.exists(metadata_path):
                try:
                    with open(metadata_path, "r", encoding="utf-8") as f:
                        metadata = json.load(f)
                except Exception:
                    metadata = {}
            datasets.append(
                {
                    "datasetId": dataset_id,
                    "taskName": metadata.get("taskName", ""),
                    "label": metadata.get("label", ""),
                    "durationSec": metadata.get("durationSec", 0.0),
                    "robotReady": robot_ready,
                    "operatorReady": upload_complete,
                    "uploadOnly": (dataset_id.startswith("uploadonly-") or (upload_complete and not robot_ready)),
                    "stuck": bool(upload_complete and not robot_ready),
                    "uploadStatus": metadata.get("uploadStatus", "pending"),
                    "uploadedAtUnixNs": int(metadata.get("uploadedAtUnixNs", 0) or 0),
                    "uploadLastError": metadata.get("uploadLastError", ""),
                    "updatedUnixNs": int(os.path.getmtime(root) * 1e9) if os.path.exists(root) else 0,
                }
            )
        state = {}
        state_path = os.path.join(self.config.cache_root, "session_state.json")
        if os.path.exists(state_path):
            try:
                with open(state_path, "r", encoding="utf-8") as f:
                    state = json.load(f)
            except Exception:
                state = {}
        return {
            "status": "ok",
            "summary": {
                "totalDatasets": len(datasets),
                "robotReadyCount": sum(1 for d in datasets if d["robotReady"]),
                "operatorReadyCount": sum(1 for d in datasets if d["operatorReady"]),
                "uploadedCount": sum(1 for d in datasets if d.get("uploadStatus") == "uploaded"),
                "uploadFailedCount": sum(1 for d in datasets if d.get("uploadStatus") == "failed"),
                "uploadPendingCount": sum(1 for d in datasets if d.get("uploadStatus") not in ("uploaded", "failed")),
                "activeDatasetId": state.get("activeDatasetId"),
                "knownSessionCount": state.get("knownCount", 0),
            },
            "datasets": datasets,
        }

    def _collect_logs(self, lines: int):
        out = {}
        for name in ("dataset_upload_server.log", "dataset_recorder.log"):
            path = os.path.join(self.config.logs_dir, name)
            rows = []
            if os.path.exists(path):
                try:
                    with open(path, "r", encoding="utf-8") as f:
                        all_lines = f.readlines()
                    rows = [ln.strip() for ln in all_lines[-lines:] if ln.strip()]
                except Exception:
                    rows = []
            out[name] = rows
        return {"status": "ok", "logs": out}

    def _reply(self, code: HTTPStatus, data):
        body = json.dumps(data).encode("utf-8")
        self.send_response(int(code))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    rospy.init_node("dataset_upload_server", anonymous=False)
    cfg = UploadServerConfig()
    UploadHandler.config = cfg
    server = ThreadingHTTPServer((cfg.host, cfg.port), UploadHandler)
    rospy.loginfo("dataset_upload_server listening on %s:%d%s", cfg.host, cfg.port, cfg.path)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
