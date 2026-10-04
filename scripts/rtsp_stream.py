"""On-demand RTSP streams for Isaac Sim cameras.

A camera costs nothing until enabled: enabling creates its render product and starts
an ffmpeg process that publishes H.264 to a MediaMTX server; disabling tears both down.
MediaMTX and ffmpeg run from the bluenviron/mediamtx:latest-ffmpeg Docker image
(host networking), so the host needs only Docker.
"""

import queue
import shutil
import socket
import subprocess
import threading
import time

MEDIAMTX_IMAGE = "bluenviron/mediamtx:latest-ffmpeg"
MEDIAMTX_CONTAINER = "isaacsim-fastlio2-rtsp"
RTSP_PORT = 8554


def _server_ready() -> bool:
    try:
        with socket.create_connection(("127.0.0.1", RTSP_PORT), timeout=0.5):
            return True
    except OSError:
        return False


class RtspServer:
    """MediaMTX container, started if nothing already serves the RTSP port."""

    def __init__(self):
        self.started = False
        if _server_ready():
            return
        subprocess.run(["docker", "rm", "-f", MEDIAMTX_CONTAINER], capture_output=True)
        result = subprocess.run(
            ["docker", "run", "-d", "--rm", "--name", MEDIAMTX_CONTAINER, "--network", "host",
             MEDIAMTX_IMAGE],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            raise RuntimeError(f"Could not start the MediaMTX RTSP server: {result.stderr.strip()}")
        self.started = True
        for _ in range(100):
            if _server_ready():
                return
            time.sleep(0.2)
        raise RuntimeError(f"MediaMTX did not open RTSP port {RTSP_PORT}")

    def close(self) -> None:
        if self.started:
            subprocess.run(["docker", "stop", "-t", "2", MEDIAMTX_CONTAINER], capture_output=True)


_server = None


def ensure_server() -> None:
    """Start MediaMTX on first use."""
    global _server
    if _server is None:
        _server = RtspServer()


def shutdown_server() -> None:
    global _server
    if _server is not None:
        _server.close()
        _server = None


class RtspCamera:
    """One USD camera streamed to rtsp://HOST:8554/<path> while enabled."""

    def __init__(self, camera_prim: str, path: str, width: int, height: int, fps: float):
        self.camera_prim = camera_prim
        self.path = path
        self.width = width
        self.height = height
        self.fps = fps
        self.active = False
        self.render_product = None
        self.annotator = None
        self.process = None
        self.writer = None
        self.frames = None
        self.next_time = 0.0

    def set_enabled(self, enabled: bool) -> None:
        if enabled and not self.active:
            self._start()
        elif not enabled and self.active:
            self._stop()

    def _start(self) -> None:
        import omni.replicator.core as rep

        ensure_server()
        self.render_product = rep.create.render_product(self.camera_prim, (self.width, self.height))
        self.annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        self.annotator.attach([self.render_product])

        ffmpeg_args = [
            "-hide_banner", "-loglevel", "warning",
            "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{self.width}x{self.height}",
            "-use_wallclock_as_timestamps", "1", "-i", "-",
            "-vf", "format=yuv420p", "-c:v", "libx264", "-preset", "ultrafast",
            "-tune", "zerolatency", "-g", str(max(1, round(self.fps))),
            "-f", "rtsp", "-rtsp_transport", "tcp", f"rtsp://127.0.0.1:{RTSP_PORT}/{self.path}",
        ]
        if shutil.which("ffmpeg"):
            command = ["ffmpeg", *ffmpeg_args]
        else:
            command = ["docker", "run", "-i", "--rm", "--network", "host",
                       "--entrypoint", "ffmpeg", MEDIAMTX_IMAGE, *ffmpeg_args]
        self.process = subprocess.Popen(command, stdin=subprocess.PIPE)
        self.frames = queue.Queue(maxsize=2)
        self.writer = threading.Thread(target=self._write_frames, daemon=True)
        self.writer.start()
        self.next_time = 0.0
        self.active = True
        print(f"RTSP stream on: rtsp://<host>:{RTSP_PORT}/{self.path}", flush=True)

    def _write_frames(self) -> None:
        while True:
            frame = self.frames.get()
            if frame is None:
                return
            try:
                self.process.stdin.write(frame)
                self.process.stdin.flush()
            except (BrokenPipeError, ValueError, OSError):
                return

    def update(self) -> None:
        """Call after every simulation_app.update(); sends a frame when one is due."""
        if not self.active:
            return
        now = time.monotonic()
        if now < self.next_time:
            return
        data = self.annotator.get_data()
        if data is None or tuple(data.shape) != (self.height, self.width, 4):
            return  # the render product has not produced a frame yet
        self.next_time = now + 1.0 / self.fps
        try:
            self.frames.put_nowait(bytes(memoryview(data).cast("B")))
        except queue.Full:
            pass  # drop the frame rather than stall the simulation

    def _stop(self) -> None:
        self.active = False
        self.frames.put(None)
        self.writer.join(timeout=2.0)
        try:
            self.process.stdin.close()
        except OSError:
            pass
        self.process.terminate()
        try:
            self.process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
        self.annotator.detach([self.render_product])
        self.render_product.destroy()
        self.annotator = self.render_product = self.process = None
        print(f"RTSP stream off: {self.path}", flush=True)

    def close(self) -> None:
        self.set_enabled(False)
