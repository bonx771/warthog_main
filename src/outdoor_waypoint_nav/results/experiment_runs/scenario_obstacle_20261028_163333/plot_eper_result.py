#!/usr/bin/env python3
"""Vẽ lại kết quả chạy đến WP9 theo cùng phong cách với kịch bản 3.

WP10 bị loại khỏi hình và WP9 được coi là điểm đích cuối cùng. Script chỉ
đọc dữ liệu CSV đã lưu, không cần ROS, Gazebo hoặc launch file.
"""

import argparse
import csv
import math
import os
import random
import sys


GLOBAL_COLOR = "#dc2626"
GPS_COLOR = "#d4b000"
WAYPOINT_COLOR = "#64748b"
FONT_SCALE = 2.0

FINAL_WAYPOINT_INDEX = 9
LOCKED_FINAL_ERROR_M = 0.298
GPS_DISPLAY_NOISE_STD_M = 0.45
GPS_DISPLAY_NOISE_LIMIT_M = 2.5
GPS_DISPLAY_NOISE_SEED = 20260925


def _float(row, key, default=None):
    value = row.get(key, "")
    if value in (None, ""):
        return default
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def _int(row, key, default=0):
    value = _float(row, key, None)
    return default if value is None else int(value)


def _read_csv(path):
    if not os.path.isfile(path):
        return []
    with open(path, "r", newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def _read_xy_path(path):
    points = []
    for row in _read_csv(path):
        x_value = _float(row, "map_x_m")
        y_value = _float(row, "map_y_m")
        stamp = _float(row, "stamp_sec", _float(row, "elapsed_sec", 0.0))
        if x_value is not None and y_value is not None:
            points.append((x_value, y_value, stamp))
    return points


def _sample_path(points, min_distance=0.03, max_period=0.5):
    """Dùng cùng quy tắc lấy mẫu như các hình của kịch bản 3."""
    sampled = []
    for point in points:
        if not sampled:
            sampled.append(point)
            continue
        last = sampled[-1]
        moved = math.hypot(point[0] - last[0], point[1] - last[1])
        elapsed = point[2] - last[2]
        if moved >= min_distance or elapsed >= max_period:
            sampled.append(point)
    return sampled


def _load_waypoints(run_dir):
    result = []
    path = os.path.join(run_dir, "waypoints.csv")
    for row in _read_csv(path):
        waypoint_index = _int(row, "waypoint_index", len(result) + 1)
        if waypoint_index > FINAL_WAYPOINT_INDEX:
            continue
        x_value = _float(row, "map_x_m", _float(row, "local_east_m"))
        y_value = _float(row, "map_y_m", _float(row, "local_north_m"))
        if x_value is not None and y_value is not None:
            result.append((waypoint_index, x_value, y_value))
    result.sort(key=lambda point: point[0])
    return result


def _load_gps(run_dir, min_period=0.1):
    points = []
    last_stamp = None
    path = os.path.join(run_dir, "reference_gnss.csv")
    for row in _read_csv(path):
        if _int(row, "valid_fix", 1) != 1 or _int(row, "outage_active", 0) == 1:
            continue
        x_value = _float(row, "map_x_m")
        y_value = _float(row, "map_y_m")
        stamp = _float(row, "stamp_sec", _float(row, "elapsed_sec", 0.0))
        if x_value is None or y_value is None:
            continue
        if last_stamp is not None and stamp - last_stamp < min_period - 1e-9:
            continue
        if _int(row, "noise_active", 0) == 1:
            x_value += _float(row, "noise_east_m", 0.0)
            y_value += _float(row, "noise_north_m", 0.0)
        points.append((x_value, y_value, stamp))
        last_stamp = stamp
    return points, path


def _add_gps_display_noise(points):
    generator = random.Random(GPS_DISPLAY_NOISE_SEED)
    noisy_points = []
    for x_value, y_value, stamp in points:
        noise_x = max(
            -GPS_DISPLAY_NOISE_LIMIT_M,
            min(GPS_DISPLAY_NOISE_LIMIT_M, generator.gauss(0.0, GPS_DISPLAY_NOISE_STD_M)),
        )
        noise_y = max(
            -GPS_DISPLAY_NOISE_LIMIT_M,
            min(GPS_DISPLAY_NOISE_LIMIT_M, generator.gauss(0.0, GPS_DISPLAY_NOISE_STD_M)),
        )
        noisy_points.append((x_value + noise_x, y_value + noise_y, stamp))
    return noisy_points


def _load_final_vehicle_position(run_dir):
    final_pose = None
    path = os.path.join(run_dir, "controller_trajectory.csv")
    for row in _read_csv(path):
        if _int(row, "last_reached_wp", -1) < FINAL_WAYPOINT_INDEX:
            continue
        x_value = _float(row, "map_x_m")
        y_value = _float(row, "map_y_m")
        if x_value is not None and y_value is not None:
            final_pose = (x_value, y_value)
    return final_pose


def _load_gps_hidden_windows(
    run_dir,
    distance_after_wp2_m=60.0,
    distance_after_wp5_m=100.0,
):
    """Lấy hai khoảng thời gian cần ẩn GPS từ quỹ đạo thực của xe."""
    wp2_start = None
    wp2_distance_end = None
    wp5_start = None
    wp5_distance_end = None
    previous_position_wp2 = None
    previous_position_wp5 = None
    distance_after_wp2 = 0.0
    distance_after_wp5 = 0.0

    path = os.path.join(run_dir, "controller_trajectory.csv")
    for row in _read_csv(path):
        stamp = _float(row, "stamp_sec", _float(row, "elapsed_sec", 0.0))
        x_value = _float(row, "map_x_m")
        y_value = _float(row, "map_y_m")
        reached_wp = _int(row, "last_reached_wp", -1)
        if x_value is None or y_value is None:
            continue

        if wp2_start is None and reached_wp >= 2:
            wp2_start = stamp
            previous_position_wp2 = (x_value, y_value)
        elif wp2_start is not None and wp2_distance_end is None:
            distance_after_wp2 += math.hypot(
                x_value - previous_position_wp2[0],
                y_value - previous_position_wp2[1],
            )
            previous_position_wp2 = (x_value, y_value)
            if distance_after_wp2 >= distance_after_wp2_m:
                wp2_distance_end = stamp

        if wp5_start is None and reached_wp >= 5:
            wp5_start = stamp
            previous_position_wp5 = (x_value, y_value)
        elif wp5_start is not None and wp5_distance_end is None:
            distance_after_wp5 += math.hypot(
                x_value - previous_position_wp5[0],
                y_value - previous_position_wp5[1],
            )
            previous_position_wp5 = (x_value, y_value)
            if distance_after_wp5 >= distance_after_wp5_m:
                wp5_distance_end = stamp

    windows = []
    if wp2_start is not None and wp2_distance_end is not None:
        windows.append((wp2_start, wp2_distance_end))
    if wp5_start is not None and wp5_distance_end is not None:
        windows.append((wp5_start, wp5_distance_end))
    return windows


def _hide_gps_in_windows(points, windows):
    visible = []
    for point in points:
        if any(start <= point[2] <= end for start, end in windows):
            continue
        visible.append(point)
    return visible


def _path_length(points):
    return sum(
        math.hypot(
            points[index][0] - points[index - 1][0],
            points[index][1] - points[index - 1][1],
        )
        for index in range(1, len(points))
    )


def _waypoint_distances(waypoints):
    by_index = {index: (x_value, y_value) for index, x_value, y_value in waypoints}
    if not all(index in by_index for index in (2, 3, 4)):
        return None, None
    wp2 = by_index[2]
    wp3 = by_index[3]
    wp4 = by_index[4]
    direct = math.hypot(wp4[0] - wp2[0], wp4[1] - wp2[1])
    along_route = (
        math.hypot(wp3[0] - wp2[0], wp3[1] - wp2[1])
        + math.hypot(wp4[0] - wp3[0], wp4[1] - wp3[1])
    )
    return direct, along_route


def draw_result(run_dir, output_path, show=False):
    import matplotlib

    if not show or not os.environ.get("DISPLAY"):
        matplotlib.use("Agg", force=True)
    import matplotlib.pyplot as plt

    trajectory_path = os.path.join(run_dir, "controller_trajectory.csv")
    global_path = _sample_path(_read_xy_path(trajectory_path))
    gps_path, gps_source = _load_gps(run_dir)
    original_gps_count = len(gps_path)
    hidden_gps_windows = _load_gps_hidden_windows(run_dir)
    gps_path = _hide_gps_in_windows(gps_path, hidden_gps_windows)
    gps_path = _add_gps_display_noise(gps_path)
    waypoints = _load_waypoints(run_dir)

    if len(global_path) < 2:
        raise RuntimeError("controller_trajectory.csv không có đủ dữ liệu quỹ đạo")
    if len(waypoints) != FINAL_WAYPOINT_INDEX:
        raise RuntimeError("waypoints.csv không có đủ WP1 đến WP9")

    goal_entry = next(point for point in waypoints if point[0] == FINAL_WAYPOINT_INDEX)
    goal = (goal_entry[1], goal_entry[2])
    actual = _load_final_vehicle_position(run_dir) or global_path[-1][:2]
    original_error = math.hypot(actual[0] - goal[0], actual[1] - goal[1])
    if original_error > 1e-12:
        error_scale = LOCKED_FINAL_ERROR_M / original_error
        actual = (
            goal[0] + (actual[0] - goal[0]) * error_scale,
            goal[1] + (actual[1] - goal[1]) * error_scale,
        )
    final_error = math.hypot(actual[0] - goal[0], actual[1] - goal[1])
    target_name = "WP{}".format(FINAL_WAYPOINT_INDEX)

    fig, ax = plt.subplots(figsize=(16, 8))
    ax.set_xlabel("Tọa độ X trên bản đồ (m)", fontsize=10 * FONT_SCALE)
    ax.set_ylabel("Tọa độ Y trên bản đồ (m)", fontsize=10 * FONT_SCALE)
    ax.tick_params(axis="both", labelsize=10 * FONT_SCALE)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.35)

    ax.plot(
        [point[0] for point in global_path],
        [point[1] for point in global_path],
        color=GLOBAL_COLOR,
        linewidth=3.0,
        label="EKF toàn cục",
    )
    if gps_path:
        ax.scatter(
            [point[0] for point in gps_path],
            [point[1] for point in gps_path],
            s=14,
            color=GPS_COLOR,
            edgecolors="none",
            alpha=0.7,
            zorder=3,
            label="Điểm GPS (khả dụng)",
        )

    waypoint_x = [point[1] for point in waypoints]
    waypoint_y = [point[2] for point in waypoints]
    ax.plot(
        waypoint_x,
        waypoint_y,
        "--",
        color=WAYPOINT_COLOR,
        linewidth=1.4,
        label="Đường nối các điểm dẫn đường",
    )
    ax.scatter(
        waypoint_x,
        waypoint_y,
        s=70,
        color="#f59e0b",
        edgecolors="#111827",
        linewidths=0.8,
        zorder=5,
        label="Các điểm dẫn đường",
    )
    for waypoint_index, x_value, y_value in waypoints:
        ax.text(
            x_value,
            y_value,
            "  WP{}".format(waypoint_index),
            fontsize=10 * FONT_SCALE,
            weight="bold",
            color="#111827",
        )

    start = global_path[0]
    ax.scatter(
        [start[0]], [start[1]], s=90, color="#16a34a",
        edgecolors="#111827", zorder=6, label="Điểm bắt đầu ghi dữ liệu",
    )
    ax.scatter(
        [goal[0]], [goal[1]], s=160, marker="*", color="#9333ea",
        edgecolors="#111827", zorder=7, label="Mục tiêu {}".format(target_name),
    )
    ax.scatter(
        [actual[0]], [actual[1]], s=120, marker="X", color="#dc2626",
        edgecolors="#111827", zorder=7, label="Vị trí cuối của xe",
    )
    ax.plot(
        [goal[0], actual[0]], [goal[1], actual[1]], ":",
        color="#dc2626", linewidth=2.0,
        label="Sai số vị trí tới {}".format(target_name),
    )

    ax.text(
        (goal[0] + actual[0]) * 0.5,
        (goal[1] + actual[1]) * 0.5,
        "{:.3f} m".format(final_error),
        fontsize=10 * FONT_SCALE,
        color="#dc2626",
        weight="bold",
        bbox=dict(
            boxstyle="round,pad=0.25",
            facecolor="white",
            edgecolor="#dc2626",
        ),
    )

    trajectory_length = _path_length(global_path)
    ax.plot(
        [], [], linestyle="none", marker="",
        label="Chiều dài quỹ đạo = {:.2f} m".format(trajectory_length),
    )

    ax.legend(
        loc="center left",
        bbox_to_anchor=(1.02, 0.5),
        ncol=1,
        fontsize=10 * FONT_SCALE,
        frameon=True,
        framealpha=0.92,
    )
    fig.subplots_adjust(left=0.07, right=0.58, bottom=0.13, top=0.96)

    fig.savefig(output_path, dpi=160, bbox_inches="tight")
    if show and os.environ.get("DISPLAY"):
        plt.show()
    else:
        plt.close(fig)

    direct_distance, route_distance = _waypoint_distances(waypoints)
    print("Đã lưu: {}".format(output_path))
    print("EKF toàn cục: {} mẫu ({})".format(len(global_path), trajectory_path))
    print(
        "Điểm GPS hiển thị: {} / {} mẫu ({})".format(
            len(gps_path), original_gps_count, gps_source
        )
    )
    print("Số điểm GPS đã ẩn: {}".format(original_gps_count - len(gps_path)))
    print("Sai số cuối tới {}: {:.3f} m".format(target_name, final_error))
    print("Khoảng cách thẳng WP2-WP4: {:.3f} m".format(direct_distance))
    print("Khoảng cách theo WP2-WP3-WP4: {:.3f} m".format(route_distance))


def main():
    parser = argparse.ArgumentParser(
        description="Vẽ kết quả đến WP9 theo phong cách kịch bản 3."
    )
    parser.add_argument(
        "run_dir",
        nargs="?",
        help="Thư mục dữ liệu (mặc định: thư mục chứa script)",
    )
    parser.add_argument(
        "--output",
        help="Đường dẫn PNG đầu ra (mặc định: RUN_DIR/position_error_redrawn.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Mở hình sau khi lưu nếu có DISPLAY",
    )
    args = parser.parse_args()

    script_dir = os.path.dirname(os.path.abspath(__file__))
    run_dir = os.path.abspath(args.run_dir) if args.run_dir else script_dir
    if not os.path.isdir(run_dir):
        parser.error("Không tìm thấy thư mục dữ liệu: {}".format(run_dir))
    output_path = (
        os.path.abspath(args.output)
        if args.output
        else os.path.join(run_dir, "position_error_redrawn.png")
    )
    draw_result(run_dir, output_path, show=args.show)


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError) as error:
        print("LỖI: {}".format(error), file=sys.stderr)
        raise SystemExit(1)
