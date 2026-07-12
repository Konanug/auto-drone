"""Minimal MJPEG-over-HTTP server, shared by every entrypoint that wants a
live annotated view (main.py, vision_test.py, ...).

Browser: http://<pi-ip>:<port>/stream — use get_local_ip() to print a real,
clickable URL instead of a placeholder.
"""
import io
import socket
import socketserver
import threading
from http import server
from threading import Condition


class StreamBuffer(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = Condition()

    def push(self, jpg_bytes):
        with self.condition:
            self.frame = jpg_bytes
            self.condition.notify_all()


def _make_handler(stream_buffer):
    class Handler(server.BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == "/":
                self.send_response(301)
                self.send_header("Location", "/stream")
                self.end_headers()
            elif self.path == "/stream":
                self.send_response(200)
                self.send_header("Content-Type",
                                  "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                try:
                    while True:
                        with stream_buffer.condition:
                            stream_buffer.condition.wait()
                            frame = stream_buffer.frame
                        self.wfile.write(
                            b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                            + frame + b"\r\n"
                        )
                except Exception:
                    pass
            else:
                self.send_error(404)

    return Handler


class MjpegServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def get_local_ip():
    """Best-effort LAN IP of this machine, for printing a clickable stream URL.

    Opens a UDP socket toward a public address without sending any packet —
    this just asks the OS which local interface/IP would be used, so it works
    fine with no internet access as long as the Pi has a LAN route.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def start_mjpeg_server(port):
    """Starts the MJPEG server in a background thread.

    Returns (httpd, stream_buffer) — push JPEG bytes to stream_buffer.push(),
    call httpd.shutdown() to stop.
    """
    stream_buffer = StreamBuffer()
    httpd = MjpegServer(("", port), _make_handler(stream_buffer))
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    return httpd, stream_buffer
