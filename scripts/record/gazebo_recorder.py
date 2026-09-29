#!/usr/bin/env python3
"""Record a top-down video of a running SwarmX Gazebo simulation.

Spawns a static overhead camera into the world, bridges its image stream to
ROS, overlays sim time / deliveries / ground-truth collisions (read from the
dashboard API) and pipes the frames to ffmpeg (H.264). Stops once every task
is delivered (plus a short tail) or at --timeout.

Frames are taken on *simulation* time (camera --rate Hz), so the video plays at
real speed x (--fps / --rate) no matter how fast the PC ran the simulation.

    python3 gazebo_recorder.py --out /out/swarmx_gazebo_top.mp4 --tasks 40
"""
import argparse
import os
import json
import subprocess
import sys
import threading
import time
import urllib.request

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def camera_sdf(x, y, z, w, h, hfov, rate, topic):
    # pitch +90 deg looks straight down; yaw +90 deg puts north at the top of the image
    return f"""<?xml version="1.0"?><sdf version="1.9"><model name="overhead_camera"><static>true</static>
<link name="link">
<sensor name="overhead" type="camera"><topic>{topic}</topic><update_rate>{rate}</update_rate><always_on>true</always_on>
<camera><horizontal_fov>{hfov}</horizontal_fov><image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
<clip><near>1.0</near><far>200</far></clip></camera></sensor></link></model></sdf>"""


class Recorder(Node):
    def __init__(self, args):
        super().__init__("swarmx_video_recorder", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.args = args
        self.status = {}
        self.frames = 0
        self.first_stamp = None
        self.done_at = None
        self.ff = subprocess.Popen(
            ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "bgr24",
             "-s", f"{args.width}x{args.height}", "-r", str(args.fps), "-i", "-",
             "-c:v", "libx264", "-preset", "medium", "-crf", "20", "-pix_fmt", "yuv420p",
             "-movflags", "+faststart", args.out], stdin=subprocess.PIPE)
        self.create_subscription(Image, args.topic, self.on_image, qos_profile_sensor_data)
        threading.Thread(target=self.poll_status, daemon=True).start()

    def poll_status(self):
        while True:
            try:
                d = json.load(urllib.request.urlopen("http://localhost:8080/api/snapshot", timeout=3))
                self.status = {"delivered": d["metrics"]["delivered"], "robots": d["metrics"]["robots_total"],
                               "sim": d.get("sim") or {}}
            except Exception:  # noqa: BLE001 - dashboard not up yet
                pass
            time.sleep(0.5)

    def on_image(self, msg: Image):
        if msg.width != self.args.width or msg.height != self.args.height:
            return
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.first_stamp is None:
            self.first_stamp = t
        s = self.status
        sim = s.get("sim", {})
        delivered = s.get("delivered", 0)
        lines = [f"SwarmX  |  {s.get('robots', self.args.robots)} robots  |  decentralized (no central server)",
                 f"sim time {t:6.1f} s   delivered {delivered}/{self.args.tasks}   "
                 f"collisions {sim.get('collisions', 0)}   closest pass "
                 f"{sim['min_separation']:.2f} m" if sim.get("min_separation") else
                 f"sim time {t:6.1f} s   delivered {delivered}/{self.args.tasks}",
                 f"Gazebo Harmonic top view  |  video speed {self.args.fps / self.args.rate:.0f}x"]
        y = 34
        for i, line in enumerate(lines):
            scale = 0.9 if i < 2 else 0.6
            (tw, th), _ = cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, 2)
            cv2.rectangle(img, (12, y - th - 10), (12 + tw + 16, y + 8), (20, 20, 20), -1)
            cv2.putText(img, line, (20, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 2, cv2.LINE_AA)
            y += th + 22
        try:
            self.ff.stdin.write(img.tobytes())
        except BrokenPipeError:
            return
        self.frames += 1
        if self.frames % (self.args.rate * 10) == 0:
            self.get_logger().info(f"{self.frames} frames, sim {t:.0f} s, delivered {delivered}/{self.args.tasks}")
        if delivered >= self.args.tasks and self.done_at is None:
            self.done_at = t
            self.get_logger().info(f"all {self.args.tasks} tasks delivered at sim {t:.1f} s")

    def finished(self):
        return self.done_at is not None and self.frames and (self.first_stamp is not None) and \
            self._last_t() - self.done_at >= self.args.tail

    def _last_t(self):
        return self.first_stamp + self.frames / self.args.rate

    def close(self):
        self.ff.stdin.close()
        self.ff.wait()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--tasks", type=int, required=True)
    ap.add_argument("--robots", type=int, default=0)
    ap.add_argument("--world", default="swarmx_warehouse")
    ap.add_argument("--topic", default="/overhead/image")
    ap.add_argument("--width", type=int, default=1600)
    ap.add_argument("--height", type=int, default=960)
    ap.add_argument("--rate", type=int, default=5, help="camera frames per simulated second")
    ap.add_argument("--fps", type=int, default=25, help="video frame rate (speed-up = fps / rate)")
    ap.add_argument("--tail", type=float, default=5.0, help="seconds to keep recording after the last delivery")
    ap.add_argument("--timeout", type=float, default=3600.0, help="wall-clock limit")
    args = ap.parse_args()

    sdf = camera_sdf(17.0, 10.0, 70.0, args.width, args.height, 0.52, args.rate, args.topic)
    # `create` overrides the model pose with its own -x/-y/-z/-R/-P/-Y (default 0): pass it here.
    # pitch +90 deg looks straight down, yaw +90 deg puts north at the top of the image
    subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", args.world, "-name", "overhead_camera",
                    "-string", sdf, "-x", "17.0", "-y", "10.0", "-z", "70.0",
                    "-R", "0", "-P", "1.5707963", "-Y", "1.5707963"], check=True)
    bridge = subprocess.Popen(["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                               f"{args.topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    rec = Recorder(args)
    t0 = time.time()
    try:
        while rclpy.ok() and not rec.finished() and time.time() - t0 < args.timeout:
            rclpy.spin_once(rec, timeout_sec=0.2)
    finally:
        rec.close()
        bridge.terminate()
        print(json.dumps({"video": args.out, "frames": rec.frames, "done_at_sim_s": rec.done_at,
                          "status": rec.status}), flush=True)
    rc = 0 if rec.done_at is not None else 1
    sys.stdout.flush()
    os._exit(rc)  # skip interpreter teardown (rclpy + OpenCV destructor order segfaults at exit)


if __name__ == "__main__":
    sys.exit(main())
