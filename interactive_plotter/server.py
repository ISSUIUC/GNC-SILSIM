#!/usr/bin/env python3
"""
Interactive Plotter Dev Server with JSON API and live updates.
 - Serves static files from repo root (so /interactive_plotter assets work)
 - GET /api/results -> JSON representation of output/results.csv
 - GET /events -> Server-Sent Events notifying when results.csv changes
 - Clean Ctrl-C shutdown
"""

import http.server
import json
import os
import signal
import sys
import threading
import time
import webbrowser
from pathlib import Path
from typing import List


PORT = 8000
CSV_PATH = Path(__file__).parent.parent / "output" / "results.csv"


class EventBroker:
    """Minimal SSE broker broadcasting 'update' events when the CSV changes."""

    def __init__(self) -> None:
        self._clients: List[http.server.BaseHTTPRequestHandler] = []
        self._lock = threading.Lock()

    def add(self, handler: http.server.BaseHTTPRequestHandler) -> None:
        with self._lock:
            self._clients.append(handler)

    def remove(self, handler: http.server.BaseHTTPRequestHandler) -> None:
        with self._lock:
            if handler in self._clients:
                self._clients.remove(handler)

    def broadcast_update(self) -> None:
        with self._lock:
            dead = []
            for h in self._clients:
                try:
                    h.wfile.write(b"data: update\n\n")
                    h.wfile.flush()
                except Exception:
                    dead.append(h)
            for h in dead:
                if h in self._clients:
                    self._clients.remove(h)


broker = EventBroker()
shutdown_event = threading.Event()


def read_csv_as_json(csv_path: Path) -> dict:
    if not csv_path.exists():
        return {"rows": [], "columns": [], "mtime": None}
    with csv_path.open("r", encoding="utf-8") as f:
        lines = [line.rstrip("\n") for line in f if line.strip()]
    if not lines:
        return {"rows": [], "columns": [], "mtime": csv_path.stat().st_mtime}
    headers = [h.strip() for h in lines[0].split(",")]
    rows = []
    for line in lines[1:]:
        parts = line.split(",")
        if len(parts) != len(headers):
            continue
        row = {}
        for h, v in zip(headers, parts):
            v = v.strip()
            if h != "fsm" and v not in ("", "nan"):
                try:
                    row[h] = float(v)
                except ValueError:
                    row[h] = v
            else:
                row[h] = v
        rows.append(row)
    return {"rows": rows, "columns": headers, "mtime": Path(csv_path).stat().st_mtime}


class RequestHandler(http.server.SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    # Disable noisy logging of each GET line
    def log_message(self, format: str, *args) -> None:
        return

    def end_headers(self) -> None:
        # CORS + disable caching so clients always fetch fresh data
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self) -> None:  # type: ignore[override]
        if self.path.startswith("/api/results"):
            payload = read_csv_as_json(CSV_PATH)
            body = json.dumps(payload).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            try:
                print(f"[API] Served /api/results (rows={len(payload.get('rows', []))})")
            except Exception:
                pass
            return

        if self.path.startswith("/events"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")  # disable proxy buffering if any
            self.end_headers()
            broker.add(self)
            try:
                print("[SSE] client connected")
            except Exception:
                pass
            try:
                # Send an initial comment to establish stream
                self.wfile.write(b": connected\n\n")
                self.wfile.flush()
                while not shutdown_event.is_set():
                    time.sleep(1)
                # fallthrough to finally
            except Exception:
                pass
            finally:
                broker.remove(self)
                try:
                    print("[SSE] client disconnected")
                except Exception:
                    pass
            return

        return super().do_GET()


def file_watcher_thread() -> None:
    last_mtime = None
    while not shutdown_event.is_set():
        try:
            if CSV_PATH.exists():
                mtime = CSV_PATH.stat().st_mtime
                if last_mtime is None:
                    last_mtime = mtime
                elif mtime != last_mtime:
                    last_mtime = mtime
                    try:
                        print(f"[WATCHER] Detected change in {CSV_PATH}")
                    except Exception:
                        pass
                    broker.broadcast_update()
        except Exception:
            pass
        shutdown_event.wait(0.5)


def main() -> None:
    parent_dir = Path(__file__).parent.parent
    os.chdir(parent_dir)

    server = http.server.ThreadingHTTPServer(("", PORT), RequestHandler)

    def handle_sigint(signum, frame):  # type: ignore[no-untyped-def]
        print("\nStopping server...")
        shutdown_event.set()
        try:
            server.shutdown()
        finally:
            server.server_close()

    signal.signal(signal.SIGINT, handle_sigint)

    watcher = threading.Thread(target=file_watcher_thread, daemon=True)
    watcher.start()

    print(f"🚀 EKF Interactive Plotter Server")
    print(f"📊 Serving at: http://localhost:{PORT}")
    print(f"📁 Directory: {parent_dir}")
    print(f"🌐 Opening browser...")
    print(f"⏹️  Press Ctrl+C to stop the server")
    print("-" * 50)

    webbrowser.open(f"http://localhost:{PORT}/interactive_plotter/")

    try:
        server.serve_forever()
    finally:
        shutdown_event.set()
        server.server_close()
        print("👋 Server stopped.")


if __name__ == "__main__":
    main()
