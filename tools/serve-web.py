#!/usr/bin/env python3
"""Serves dist/web locally with the cross-origin isolation headers the web
app's serial link needs (the service worker adds them on GitHub Pages).

    tools/serve-web.py [port]
"""
import http.server
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent / "dist" / "web"


class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "require-corp")
        self.send_header("Cross-Origin-Resource-Policy", "same-origin")
        self.send_header("Cache-Control", "no-cache")
        super().end_headers()


port = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
print(f"Serving {ROOT} on http://localhost:{port}")
http.server.ThreadingHTTPServer(("127.0.0.1", port), Handler).serve_forever()
