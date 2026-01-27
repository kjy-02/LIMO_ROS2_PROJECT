#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class YoloDepthToMapNoTF(Node):
    def __init__(self):
        super().__init__('yolo_depth_to_map_notf')

        self.bridge = CvBridge()

        # === Subscribers ===
        self.sub_color = self.create_subscription(
            Image,
            "/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.sub_depth = self.create_subscription(
            Image,
            "/camera/depth/image_rect_raw",
            self.depth_callback,
            10
        )
        self.sub_info = self.create_subscription(
            CameraInfo,
            "/camera/color/camera_info",
            self.info_callback,
            10
        )
        self.sub_amcl = self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            10
        )

        self.robot_pose = None
        self.depth_image = None

        # === Marker Publisher ===
        self.marker_pub = self.create_publisher(Marker, "detected_objects", 10)

        # === YOLO Load ===
        weights = "/home/agilex/limo21/src/yolov3.weights"
        cfg = "/home/agilex/limo21/src/yolov3.cfg"
        names = "/home/agilex/limo21/src/coco.names"

        self.net = cv2.dnn.readNet(weights, cfg)
        with open(names, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        layer_names = self.net.getLayerNames()
        unconnected = self.net.getUnconnectedOutLayers()
        self.output_layers = [layer_names[i - 1] for i in unconnected.flatten()]

        # === Intrinsics ===
        self.fx = self.fy = None
        self.cx = self.cy = None

        # === Camera offset ===
        self.camera_offset = np.array([0.1, 0.0, 0.1], dtype=float)

        # === RealSense camera to base transform ===
        self.R_cam_to_base = np.array([
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0]
        ], dtype=float)

        self.get_logger().info("YOLO + Depth → Map Node Started")

    # =====================================================
    # Quaternion → Rotation matrix
    # =====================================================
    def quat_to_rot_matrix(self, qx, qy, qz, qw):
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm == 0:
            self.get_logger().warn("quaternion norm = 0 → Identity matrix 사용")
            return np.eye(3)

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        xx = qx*qx
        yy = qy*qy
        zz = qz*qz
        xy = qx*qy
        xz = qx*qz
        yz = qy*qz
        wx = qw*qx
        wy = qw*qy
        wz = qw*qz

        return np.array([
            [1 - 2*(yy + zz), 2*(xy - wz),     2*(xz + wy)],
            [2*(xy + wz),     1 - 2*(xx + zz), 2*(yz - wx)],
            [2*(xz - wy),     2*(yz + wx),     1 - 2*(xx + yy)]
        ], dtype=float)

    # =====================================================
    # Callbacks
    # =====================================================
    def info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.get_logger().info(
            f"📷 Camera info loaded: fx={self.fx:.1f}, fy={self.fy:.1f}"
        )

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.get_logger().debug("✔ Depth image updated")

    def amcl_callback(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        self.robot_pose = {
            'pos': np.array([p.x, p.y, p.z]),
            'quat': (q.x, q.y, q.z, q.w)
        }

        self.get_logger().debug(
            f"📍 AMCL pose updated: x={p.x:.2f}, y={p.y:.2f}, "
            f"yaw(q)=({q.x:.2f},{q.y:.2f},{q.z:.2f},{q.w:.2f})"
        )

    # =====================================================
    # 1) YOLO 탐지
    # =====================================================
    def detect_objects(self, frame):
        self.get_logger().debug("🔎 YOLO detection 시작")

        h, w, _ = frame.shape
        blob = cv2.dnn.blobFromImage(
            frame, 0.00392, (416, 416),
            (0, 0, 0), True, crop=False
        )
        self.net.setInput(blob)
        outs = self.net.forward(self.output_layers)

        class_ids, confidences, boxes = [], [], []

        for out in outs:
            for det in out:
                scores = det[5:]
                class_id = int(np.argmax(scores))
                conf = float(scores[class_id])

                if conf > 0.5:
                    cx = int(det[0] * w)
                    cy = int(det[1] * h)
                    bw = int(det[2] * w)
                    bh = int(det[3] * h)

                    x = int(cx - bw/2)
                    y = int(cy - bh/2)

                    boxes.append([x, y, bw, bh])
                    confidences.append(conf)
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)
        self.get_logger().info(f"📦 YOLO detected {len(indexes)} objects")

        return indexes, boxes, class_ids

    # =====================================================
    # 2) Depth → 카메라 좌표 변환
    # =====================================================
    def compute_depth_point(self, u, v):
        if self.depth_image is None:
            self.get_logger().warn("⚠ depth_image 없음 (compute_depth_point)")
            return None

        h_d, w_d = self.depth_image.shape[:2]

        # u, v를 depth 이미지 범위로 클램핑
        u = max(0, min(u, w_d - 1))
        v = max(0, min(v, h_d - 1))

        v0 = max(0, v - 2)
        v1 = min(h_d, v + 3)
        u0 = max(0, u - 2)
        u1 = min(w_d, u + 3)

        patch = self.depth_image[v0:v1, u0:u1].astype(float)

        if patch.size == 0:
            self.get_logger().warn("⚠ depth patch size = 0 → skip")
            return None

        Z_raw = np.median(patch)

        if np.isnan(Z_raw) or Z_raw <= 0:
            self.get_logger().warn(f"⚠ invalid depth Z={Z_raw}")
            return None

        # RealSense 16UC1(mm) 고려
        Z = Z_raw / 1000.0 if Z_raw > 20 else Z_raw

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        self.get_logger().debug(
            f"📌 Camera pos: X={X:.2f}, Y={Y:.2f}, Z={Z:.2f}"
        )

        return np.array([X, Y, Z])

    # =====================================================
    # 3) base_link → map 변환
    # =====================================================
    def transform_to_map(self, P_cam):
        P_base = self.R_cam_to_base.dot(P_cam) + self.camera_offset
        self.get_logger().debug(f"⚙ base coords: {P_base}")

        pos = self.robot_pose['pos']
        q = self.robot_pose['quat']
        R_map_base = self.quat_to_rot_matrix(*q)

        P_map = R_map_base.dot(P_base) + pos

        self.get_logger().info(f"🌐 Map coords: {P_map}")

        return P_map

    # =====================================================
    # Main Image Callback
    # =====================================================
    def image_callback(self, msg):
        if self.depth_image is None:
            self.get_logger().warn("⏳ depth_image 없음 → skip frame")
            return

        if self.fx is None:
            self.get_logger().warn("⏳ camera_info 없음 → skip frame")
            return

        if self.robot_pose is None:
            self.get_logger().warn("⏳ AMCL pose 없음 → skip frame")
            return

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        indexes, boxes, class_ids = self.detect_objects(frame)

        if len(indexes) == 0:
            self.get_logger().debug("🔕 YOLO 결과 없음")
            return

        for idx in indexes.flatten():
            i = int(idx)  # numpy.int64 → int

            x, y, w, h = boxes[i]

            # bbox 중심
            u = int(x + w * 0.5)
            v = int(y + h * 0.5)

            P_cam = self.compute_depth_point(u, v)
            if P_cam is None:
                continue

            P_map = self.transform_to_map(P_cam)
            self.publish_marker(P_map, class_ids[i], i)

    # =====================================================
    # Marker Publish
    # =====================================================
    def publish_marker(self, P_map, class_id, marker_id):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "yolo_objects"
        marker.id = int(marker_id)  # ★ 여기 int() 캐스팅 추가
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(P_map[0])
        marker.pose.position.y = float(P_map[1])
        marker.pose.position.z = float(P_map[2])

        marker.scale.x = marker.scale.y = marker.scale.z = 0.2

        color = np.random.uniform(0, 1, 3)
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = 1.0

        self.marker_pub.publish(marker)
        self.get_logger().info(f"📍 Marker published at map={P_map}")


def main(args=None):
    rclpy.init(args=args)
    node = YoloDepthToMapNoTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("노드 종료 중...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
