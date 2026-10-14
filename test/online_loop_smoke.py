#!/usr/bin/env python3
"""Exercise the real STD -> ICP -> iSAM2 ROS node on a repeated static scene.

Run after sourcing the package overlay. An optional binary XYZI PCD supplies a
real scene; the default scene is deterministic and generated in memory.
"""
import argparse
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from super_lio_slam.msg import KeyFrame
from visualization_msgs.msg import Marker, MarkerArray


def scene_points(pcd):
    if pcd:
        with open(pcd, 'rb') as stream:
            header = {}
            while True:
                line = stream.readline().decode('ascii').strip()
                if not line or line.startswith('#'):
                    continue
                key, *values = line.split()
                header[key] = values
                if key == 'DATA':
                    break
            if header['DATA'] != ['binary'] or set(header['SIZE']) != {'4'}:
                raise ValueError('Test PCD must use binary float32 scalar fields')
            fields = header['FIELDS']
            raw = np.frombuffer(stream.read(), dtype='<f4').reshape(-1, len(fields))
            return raw[:, [fields.index(name) for name in ('x', 'y', 'z', 'intensity')]].copy()
    rng = np.random.default_rng(47)
    planes = []
    for lo, hi in [((-15, -12, 0), (15, 12, 5)),
                   ((-8, -5, 0), (-5, -2, 3)), ((1, -7, 0), (4, -3, 2)),
                   ((7, 3, 0), (10, 6, 4)), ((-4, 5, 0), (-1, 8, 2.5))]:
        lo, hi = np.array(lo), np.array(hi)
        for axis in range(3):
            for side in (lo[axis], hi[axis]):
                xyz = rng.uniform(lo, hi, (4000, 3))
                xyz[:, axis] = side
                planes.append(np.column_stack((xyz, np.ones(len(xyz)))))
    return np.concatenate(planes).astype('<f4')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', required=True, help='loop_backend_node executable')
    parser.add_argument('--pcd', help='Optional binary PCD scene')
    parser.add_argument('--moving', action='store_true', help='Use nonzero true translation and rotation')
    parser.add_argument('--reject-icp', action='store_true', help='Force the ICP minimum-point gate to reject candidates')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '127')
    os.environ.setdefault('ROS_LOCALHOST_ONLY', '1')
    os.environ.setdefault('ROS_LOG_DIR', '/tmp/super_lio_slam_test_logs')
    points = scene_points(args.pcd)
    points = points[np.isfinite(points).all(axis=1)]
    log = tempfile.NamedTemporaryFile(prefix='super_lio_loop_test_', suffix='.log', delete=False)
    command = [args.backend, '--ros-args', '-p', 'skip_near_num:=2',
               '-p', 'loop_min_interval:=1', '-p', 'icp_submap_half_width:=0',
               '-p', 'icp_max_translation_correction:=5.0',
               '-p', 'graph_loop_translation_sigma:=0.02',
               '-p', 'map_publish_stride:=1']
    if args.reject_icp:
        command += ['-p', 'icp_min_points:=20000']
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('online_loop_smoke')
    received, loops, std_triangles = [], [], []
    node.create_subscription(Odometry, '/lio_slam/odom', received.append, 10)
    node.create_subscription(MarkerArray, '/lio_slam/loop_markers', loops.append, 10)
    node.create_subscription(MarkerArray, '/lio_slam/std_triangles', std_triangles.append, 10)
    publisher = node.create_publisher(KeyFrame, '/lio_slam/keyframe', 10)
    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publisher.get_subscription_count(), 'Backend subscription unavailable'
        for index in range(5):
            msg = KeyFrame()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'world'
            msg.id = index
            # A grid-aligned 90-degree revisit preserves the procedural scene's
            # corners while exposing an inverted ICP/BetweenFactor transform.
            true_translation = np.array([2.0, 2.0, 0.0]) if args.moving and index >= 3 else np.zeros(3)
            true_yaw = np.pi / 2 if args.moving and index >= 3 else 0.0
            raw_yaw = true_yaw + (0.02 * index if args.moving else 0.0)
            msg.odom_pose.position.x = float(true_translation[0] + 0.25 * index)
            msg.odom_pose.position.y = float(true_translation[1])
            msg.odom_pose.orientation.z = float(np.sin(raw_yaw / 2))
            msg.odom_pose.orientation.w = float(np.cos(raw_yaw / 2))
            c, s = np.cos(true_yaw), np.sin(true_yaw)
            rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            local_points = points.copy()
            local_points[:, :3] = (points[:, :3] - true_translation) @ rotation
            cloud = PointCloud2()
            cloud.header.stamp = msg.header.stamp
            cloud.header.frame_id = 'imu'
            cloud.height, cloud.width = 1, len(points)
            cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                            for i, name in enumerate(('x', 'y', 'z', 'intensity'))]
            cloud.is_bigendian = False
            cloud.point_step, cloud.row_step = 16, 16 * len(points)
            cloud.is_dense = True
            cloud.data = np.ascontiguousarray(local_points, dtype='<f4').tobytes()
            msg.cloud = cloud
            publisher.publish(msg)
            deadline = time.monotonic() + 30
            while len(received) <= index and time.monotonic() < deadline:
                assert process.poll() is None, 'Backend exited unexpectedly'
                rclpy.spin_once(node, timeout_sec=0.1)
            assert len(received) > index, f'No optimized odometry for keyframe {index}'
        deadline = time.monotonic() + 2
        while not loops and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        deadline = time.monotonic() + 2
        while len(std_triangles) < 5 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(std_triangles) >= 5, 'Missing online STD triangle markers'
        edges, vertices = std_triangles[-1].markers
        assert edges.header.frame_id == 'map' and edges.type == Marker.LINE_LIST
        assert vertices.header.frame_id == 'map' and vertices.type == Marker.SPHERE_LIST
        assert 0 < len(edges.points) <= 600 and len(edges.points) % 6 == 0
        assert len(vertices.points) * 2 == len(edges.points)
        has_loop = bool(loops and any(m.points for a in loops for m in a.markers))
        pose = received[-1].pose.pose
        corrected = float(np.linalg.norm(np.array([pose.position.x, pose.position.y, pose.position.z]) - true_translation))
        if args.reject_icp:
            assert not has_loop, 'Rejected ICP candidate created a loop marker'
            assert abs(corrected - 1.0) < 0.01, f'Rejected ICP changed odometry: {corrected}'
            print('PASS: ICP rejection left the 1.000 m raw drift unchanged')
        else:
            assert has_loop, 'No accepted loop'
            assert corrected < 0.5, f'PGO did not reduce artificial 1 m drift: {corrected}'
            if args.moving:
                estimated_yaw = 2 * np.arctan2(pose.orientation.z, pose.orientation.w)
                assert abs(estimated_yaw - true_yaw) < 0.06, 'Incorrect loop rotation direction'
            print(f'PASS: 5 keyframes, accepted loop, 1.000 m raw drift -> {corrected:.4f} m corrected')
        log.flush()
        diagnostic_log = Path(log.name).read_text()
        assert 'Loop search input=' in diagnostic_log, 'Missing STD search diagnostics'
        assert 'hash_candidates=' in diagnostic_log and 'plane_score=' in diagnostic_log, \
            'Missing STD filter or geometry metrics'
        if args.reject_icp:
            assert 'rejected: insufficient points' in diagnostic_log, \
                'Missing ICP gate diagnostic'
        print(f'Backend log: {log.name}')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        print(Path(log.name).read_text())


if __name__ == '__main__':
    main()
