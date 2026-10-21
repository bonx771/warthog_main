#!/usr/bin/env python3
import json
import os
import signal
import subprocess
import shlex
import threading
import time
import unicodedata
from collections import deque
from pathlib import Path

import yaml

import rospy
from std_msgs.msg import String

COMMAND_TOPIC = "/hmi/launch_command"
MAP_COMMAND_TOPIC = "/hmi/map_command"
MAP_LIST_TOPIC = "/hmi/map_list"
STATUS_TOPIC = "/hmi/launch_status"
WORKSPACE_SETUP = "/home/cuong/warthog_full/devel/setup.bash"
WORKSPACE_SETUP_EKF = "/home/cuong/warthog/devel/setup.bash"
MAP_LAUNCHES = ("save_aloam_2d_map", "helios32_raw_map_navigation_trajectory")


def normalize_map_name(value):
    if not isinstance(value, str):
        raise ValueError("Cần chọn hoặc nhập tên bản đồ.")
    name = unicodedata.normalize("NFC", value.strip())
    if not name or len(name) > 80 or not name[0].isalnum() or not all(
        character.isalnum() or character in " _-" for character in name
    ):
        raise ValueError("Tên bản đồ dài 1–80 ký tự, chỉ gồm chữ, số, dấu cách, _ hoặc -.")
    return name


LAUNCH_MAP = {
    # Khởi động EKF
    "outdoor_waypoint_nav": [
        "bash",                                      
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav amcl_odom_tf.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    # Khởi động Lidar
    "rslidar_start": [
        "bash",
        "-lc",
        "source {} && roslaunch rslidar_sdk start.launch".format(WORKSPACE_SETUP),
    ],

    # Quét địa hình
    "aloam_velodyne_HDL_32": [
        "bash",
        "-lc",
        "source {} && roslaunch aloam_velodyne aloam_velodyne_HDL_32.launch".format(
            WORKSPACE_SETUP
        ),
    ],

    # Lưu bản đồ 2D
    "save_aloam_2d_map": [
        "bash",
        "-lc",
        "source {} && roslaunch export_2d_map save_aloam_2d_map.launch".format(
            WORKSPACE_SETUP
        ),
    ],

    # Chạy đúng bản đồ được chọn ở vali; map_name được thêm trong start_launch.
    "helios32_raw_map_navigation_trajectory": [
        "bash",
        "-lc",
        "source {} && roslaunch rslidar_sdk helios32_raw_map_navigation_trajectory.launch".format(WORKSPACE_SETUP),
    ],

    "imu_start_calib": [
        "bash",                                      
        "-lc",
        "source {} && roslaunch imu_pkg start_calib.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],

}


class LaunchManager:
    def __init__(self):
        default_machine_name = "cuong"
        self.machine_name = rospy.get_param("~machine_name", default_machine_name)
        self.machine_name = self.machine_name.strip()
        self.processes = {}
        self.map_jobs = {}
        self.log_files = {}
        self.lock = threading.RLock()
        self.request_ids = set()
        self.request_order = deque()
        # Cùng thư mục mà $(find warthog_slam)/maps của hai launch sử dụng.
        default_map_directory = Path(__file__).resolve().parents[2] / "warthog_slam" / "maps"
        self.map_directory = Path(rospy.get_param("~map_directory", str(default_map_directory))).expanduser().resolve()
        self.save_timeout = float(rospy.get_param("~save_timeout", 120.0))

        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=20)
        self.map_list_pub = rospy.Publisher(MAP_LIST_TOPIC, String, queue_size=10, latch=True)
        self.command_sub = rospy.Subscriber(
            COMMAND_TOPIC, String, self.handle_command, queue_size=20
        )
        self.map_command_sub = rospy.Subscriber(
            MAP_COMMAND_TOPIC, String, self.handle_command, queue_size=20
        )
        self.process_timer = rospy.Timer(rospy.Duration(1.0), self.check_processes)

        rospy.on_shutdown(self.stop_all)
        rospy.loginfo(
            "Launch manager ready: machine_name=%s topic=%s",
            self.machine_name,
            COMMAND_TOPIC,
        )

    def publish_status(self, payload):
        payload["machine_name"] = self.machine_name
        payload["timestamp"] = rospy.Time.now().to_sec()
        self.status_pub.publish(String(data=json.dumps(payload)))

    def matches_target(self, target_machine):
        target = str(target_machine or "").strip()
        if target in ("*", "all"):
            return True

        return target == self.machine_name

    def map_path(self, map_name):
        return self.map_directory / (normalize_map_name(map_name) + ".yaml")

    def valid_map(self, map_name):
        path = self.map_path(map_name)
        if path.is_symlink() or not path.is_file():
            return False
        try:
            with path.open(encoding="utf-8") as stream:
                metadata = yaml.safe_load(stream)
            if not isinstance(metadata, dict) or not isinstance(metadata.get("image"), str):
                return False
            image = (self.map_directory / metadata["image"]).resolve()
            return image.parent == self.map_directory and image.is_file() and image.stat().st_size > 0
        except (OSError, ValueError, yaml.YAMLError):
            return False

    def publish_map_list(self, request_id=""):
        maps = []
        error = None
        saving = self.map_jobs.get("save_aloam_2d_map", {}).get("map_name")
        try:
            for path in self.map_directory.glob("*.yaml"):
                try:
                    if path.stem != saving and self.valid_map(path.stem):
                        maps.append({"name": path.stem})
                except ValueError:
                    continue
        except OSError as exc:
            error = "Không đọc được thư mục bản đồ: {}".format(exc)
        payload = {
            "event": "map_list", "machine_name": self.machine_name,
            "request_id": request_id, "maps": sorted(maps, key=lambda entry: entry["name"]),
        }
        if error:
            payload["error"] = error
        self.map_list_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))

    @staticmethod
    def terminate_process(process):
        if process.poll() is not None:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            try:
                os.killpg(os.getpgid(process.pid), sig)
                process.wait(timeout=3)
                return
            except ProcessLookupError:
                return
            except subprocess.TimeoutExpired:
                continue

    def release_process(self, launch_name):
        self.processes.pop(launch_name, None)
        log_file = self.log_files.pop(launch_name, None)
        if log_file:
            log_file.close()

    def cleanup_exited_process(self, launch_name):
        process = self.processes.get(launch_name)
        if not process:
            return

        return_code = process.poll()
        job = self.map_jobs.get(launch_name, {})
        if launch_name == "save_aloam_2d_map" and job:
            complete = self.valid_map(job["map_name"])
            timed_out = time.monotonic() - job["started_at"] > self.save_timeout
            if complete or timed_out or return_code is not None:
                self.terminate_process(process)
                self.release_process(launch_name)
                self.map_jobs.pop(launch_name, None)
                self.publish_status({
                    "event": "saved" if complete else "save_failed", "launch": launch_name,
                    "request_id": job["request_id"], "map_name": job["map_name"],
                    "message": "Đã lưu bản đồ." if complete else "Chưa lưu đủ file bản đồ. Kiểm tra dữ liệu quét và log trên robot.",
                })
                self.publish_map_list()
            return
        if return_code is not None:
            self.release_process(launch_name)
            self.map_jobs.pop(launch_name, None)
            self.publish_status(
                {
                    "event": "exited",
                    "launch": launch_name,
                    "return_code": return_code,
                    "request_id": job.get("request_id", ""),
                    "map_name": job.get("map_name", ""),
                }
            )

    def check_processes(self, _event):
        with self.lock:
            for launch_name in list(self.processes.keys()):
                self.cleanup_exited_process(launch_name)

    def handle_command(self, msg):
        with self.lock:
            self._handle_command(msg)

    def _handle_command(self, msg):
        try:
            payload = json.loads(msg.data)
        except ValueError:
            rospy.logwarn("Invalid launch command JSON: %s", msg.data)
            return

        if not isinstance(payload, dict):
            return
        target_machine = payload.get("target_machine")
        if not self.matches_target(target_machine):
            return

        action = str(payload.get("action", "")).strip().lower()
        launch_name = str(payload.get("launch", "")).strip()
        request_id = payload.get("request_id", "")
        # Một lệnh có thể tới từ nhiều rosbridge: chỉ xử lý mỗi request_id một lần.
        if isinstance(request_id, str) and request_id:
            if request_id in self.request_ids:
                return
            self.request_ids.add(request_id)
            self.request_order.append(request_id)
            if len(self.request_order) > 1000:
                self.request_ids.remove(self.request_order.popleft())

        if action == "list":
            self.publish_map_list(request_id)
            return

        if action == "start":
            map_name = payload.get("map_name", "")
            self.start_launch(launch_name, request_id, map_name)
            return

        if action == "stop":
            self.stop_launch(launch_name, request_id)
            return

        self.publish_status(
            {
                "event": "rejected",
                "reason": "unknown_action",
                "action": action,
                "launch": launch_name,
                "request_id": request_id,
            }
        )

    def start_launch(self, launch_name, request_id="", map_name=""):
        self.cleanup_exited_process(launch_name)

        if launch_name not in LAUNCH_MAP:
            self.publish_status(
                {
                    "event": "rejected",
                    "reason": "launch_not_whitelisted",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        if launch_name in MAP_LAUNCHES:
            try:
                map_name = normalize_map_name(map_name)
            except ValueError as error:
                self.publish_status({"event": "rejected", "launch": launch_name,
                                     "request_id": request_id, "message": str(error)})
                return

        if launch_name in self.processes:
            current_map = self.map_jobs.get(launch_name, {}).get("map_name")
            if current_map and current_map != map_name:
                self.publish_status({"event": "rejected", "launch": launch_name, "request_id": request_id,
                                     "message": "Tác vụ đang chạy bản đồ khác. Nhấn STOP trước khi đổi bản đồ."})
                return
            self.publish_status(
                {
                    "event": "already_running",
                    "launch": launch_name,
                    "pid": self.processes[launch_name].pid,
                    "request_id": request_id,
                    "map_name": current_map,
                }
            )
            return

        command = list(LAUNCH_MAP[launch_name])

        if launch_name in MAP_LAUNCHES:
            try:
                path = self.map_path(map_name)
                if launch_name == "save_aloam_2d_map":
                    self.map_directory.mkdir(parents=True, exist_ok=True)
                    if os.path.lexists(str(path)) or os.path.lexists(str(path.with_suffix(".pgm"))):
                        raise ValueError("Tên bản đồ đã tồn tại. Hãy chọn tên khác.")
                elif not self.valid_map(map_name):
                    raise ValueError("Không tìm thấy bộ file bản đồ đã chọn trên robot.")
            except (OSError, ValueError) as error:
                self.publish_status({"event": "rejected", "launch": launch_name,
                                     "request_id": request_id, "message": str(error)})
                return
            # Cả hai launch cùng nhận map_name:=<tên từ vali>; thư mục do robot quản lý.
            # quote toàn bộ đối số để tên có dấu cách không bị tách hoặc thực thi shell.
            command[2] += " " + shlex.quote("map_name:=" + map_name)
            command[2] += " " + shlex.quote("map_dir:=" + str(self.map_directory))
        log_path = "/tmp/hmi_launch_{}.log".format(launch_name)
        log_file = None
        try:
            log_file = open(log_path, "a")
            rospy.loginfo("Starting launch %s: %s", launch_name, " ".join(command))
            process = subprocess.Popen(command, stdout=log_file, stderr=subprocess.STDOUT, preexec_fn=os.setsid)
        except OSError as error:
            if log_file:
                log_file.close()
            self.publish_status({"event": "start_failed", "launch": launch_name,
                                 "request_id": request_id, "message": str(error)})
            return
        self.processes[launch_name] = process
        self.log_files[launch_name] = log_file
        if launch_name in MAP_LAUNCHES:
            self.map_jobs[launch_name] = {"map_name": map_name, "request_id": request_id,
                                        "started_at": time.monotonic()}

        self.publish_status(
            {
                "event": "saving" if launch_name == "save_aloam_2d_map" else "started",
                "launch": launch_name,
                "pid": process.pid,
                "log_path": log_path,
                "request_id": request_id,
                "map_name": map_name if launch_name in MAP_LAUNCHES else "",
            }
        )

    def stop_launch(self, launch_name, request_id=""):
        self.cleanup_exited_process(launch_name)

        process = self.processes.get(launch_name)
        if not process:
            self.publish_status(
                {
                    "event": "not_running",
                    "launch": launch_name,
                    "request_id": request_id,
                }
            )
            return

        rospy.loginfo("Stopping launch %s pid=%s", launch_name, process.pid)
        self.terminate_process(process)
        self.release_process(launch_name)
        job = self.map_jobs.pop(launch_name, {})

        self.publish_status(
            {
                "event": "stopped",
                "launch": launch_name,
                "request_id": request_id,
                "map_name": job.get("map_name", ""),
            }
        )

    def stop_all(self):
        with self.lock:
            for launch_name in list(self.processes.keys()):
                self.stop_launch(launch_name)


if __name__ == "__main__":
    rospy.init_node("hmi_launch_manager")
    LaunchManager()
    rospy.spin()
