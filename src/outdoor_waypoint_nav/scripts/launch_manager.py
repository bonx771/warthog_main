#!/usr/bin/env python3
import json
import os
import signal
import subprocess
import threading
from collections import deque

import rospy
from std_msgs.msg import String

COMMAND_TOPIC = "/hmi/launch_command"
STATUS_TOPIC = "/hmi/launch_status"
WORKSPACE_SETUP = "/home/cuong/warthog_full/devel/setup.bash"
WORKSPACE_SETUP_EKF = "/home/cuong/warthog/devel/setup.bash"


LAUNCH_MAP = {
    # Khởi động EKF
    "outdoor_waypoint_nav": [
        "bash",                                      
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav amcl_odom_tf.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    # Navigation và scenario vật cản nằm trong workspace warthog.
    "outdoor_waypoint_obstacle": [
        "bash",
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav outdoor_waypoint_obstacle.launch".format(
            WORKSPACE_SETUP_EKF
        ),
    ],
    "scenario_obstacle": [
        "bash",
        "-lc",
        "source {} && roslaunch outdoor_waypoint_nav scenario_obstacle.launch".format(
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
    "aloam_rtk_mapping": [
        "bash",
        "-lc",
        "source {} && roslaunch aloam_gps_localization aloam_rtk_mapping.launch".format(
            WORKSPACE_SETUP
        ),
    ],

    # Lưu bản đồ 2D
    "save_aloam_2d_map_rtk": [
        "bash",
        "-lc",
        "source {} && roslaunch aloam_gps_localization save_aloam_2d_map_rtk.launch".format(
            WORKSPACE_SETUP
        ),
    ],

    # Chạy trajectory với map RTK mặc định của launch.
    "helios32_rtk_map_navigation_trajectory": [
        "bash",
        "-lc",
        "source {} && roslaunch aloam_gps_localization helios32_rtk_map_navigation_trajectory.launch".format(WORKSPACE_SETUP),
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
        self.launch_request_ids = {}
        self.log_files = {}
        self.lock = threading.RLock()
        self.request_ids = set()
        self.request_order = deque()

        self.status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=20)
        self.command_sub = rospy.Subscriber(
            COMMAND_TOPIC, String, self.handle_command, queue_size=20
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
        self.launch_request_ids.pop(launch_name, None)
        log_file = self.log_files.pop(launch_name, None)
        if log_file:
            log_file.close()

    def cleanup_exited_process(self, launch_name):
        process = self.processes.get(launch_name)
        if not process:
            return

        return_code = process.poll()
        if return_code is not None:
            request_id = self.launch_request_ids.get(launch_name, "")
            self.release_process(launch_name)
            self.publish_status(
                {
                    "event": "exited",
                    "launch": launch_name,
                    "return_code": return_code,
                    "request_id": request_id,
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

        if action == "start":
            self.start_launch(launch_name, request_id)
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

    def start_launch(self, launch_name, request_id=""):
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

        if launch_name in self.processes:
            self.publish_status(
                {
                    "event": "already_running",
                    "launch": launch_name,
                    "pid": self.processes[launch_name].pid,
                    "request_id": request_id,
                }
            )
            return

        command = list(LAUNCH_MAP[launch_name])

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
        self.launch_request_ids[launch_name] = request_id

        self.publish_status(
            {
                "event": "started",
                "launch": launch_name,
                "pid": process.pid,
                "log_path": log_path,
                "request_id": request_id,
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

        self.publish_status(
            {
                "event": "stopped",
                "launch": launch_name,
                "request_id": request_id,
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
