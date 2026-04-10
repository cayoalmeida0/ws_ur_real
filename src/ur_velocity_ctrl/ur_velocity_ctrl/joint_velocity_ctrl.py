#!/usr/bin/env python3

from __future__ import annotations

from typing import List, Optional, Tuple
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray


class JointVelocityCtrl(Node):
    """
    Controle cartesiano de pose para o UR3e real.

    Usa:
    - /joint_states para obter q
    - /tcp_pose_broadcaster/pose para pose medida do TCP
    - Jacobiano geométrico completo (linear + angular)
    - /forward_velocity_controller/commands para enviar qdot
    """

    def __init__(self) -> None:
        super().__init__("joint_velocity_ctrl")

        # =========================
        # Parâmetros
        # =========================
        self.declare_parameter("command_topic", "/forward_velocity_controller/commands")
        self.declare_parameter("publish_rate", 50.0)

        # Alvo cartesiano
        self.declare_parameter("x", 0.20)
        self.declare_parameter("y", 0.20)
        self.declare_parameter("z", 0.25)
        self.declare_parameter("roll", math.pi)
        self.declare_parameter("pitch", 0.0)
        self.declare_parameter("yaw", 0.0)

        # Ganhos
        self.declare_parameter("kp_pos", 0.4)
        self.declare_parameter("kp_ori", 0.6)
        self.declare_parameter("damping", 0.08)
        self.declare_parameter("max_velocity", 0.03)

        # Tolerâncias
        self.declare_parameter("pos_tolerance", 0.005)
        self.declare_parameter("ori_tolerance", 0.05)

        # Modo teste articular
        self.declare_parameter("test_joint", 0)
        self.declare_parameter("test_velocity", 0.0)
        self.declare_parameter("use_joint_test", False)

        self.command_topic = str(
            self.get_parameter("command_topic").get_parameter_value().string_value
        )
        self.publish_rate = float(
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )

        self.target_position = np.array(
            [
                float(self.get_parameter("x").get_parameter_value().double_value),
                float(self.get_parameter("y").get_parameter_value().double_value),
                float(self.get_parameter("z").get_parameter_value().double_value),
            ],
            dtype=float,
        )

        roll = float(self.get_parameter("roll").get_parameter_value().double_value)
        pitch = float(self.get_parameter("pitch").get_parameter_value().double_value)
        yaw = float(self.get_parameter("yaw").get_parameter_value().double_value)
        self.target_rotation = self.rpy_to_rotation_matrix(roll, pitch, yaw)

        self.kp_pos = float(
            self.get_parameter("kp_pos").get_parameter_value().double_value
        )
        self.kp_ori = float(
            self.get_parameter("kp_ori").get_parameter_value().double_value
        )
        self.damping = float(
            self.get_parameter("damping").get_parameter_value().double_value
        )
        self.max_velocity = float(
            self.get_parameter("max_velocity").get_parameter_value().double_value
        )

        self.pos_tolerance = float(
            self.get_parameter("pos_tolerance").get_parameter_value().double_value
        )
        self.ori_tolerance = float(
            self.get_parameter("ori_tolerance").get_parameter_value().double_value
        )

        self.test_joint = int(
            self.get_parameter("test_joint").get_parameter_value().integer_value
        )
        self.test_velocity = float(
            self.get_parameter("test_velocity").get_parameter_value().double_value
        )
        self.use_joint_test = bool(
            self.get_parameter("use_joint_test").get_parameter_value().bool_value
        )

        self.joint_names_expected: List[str] = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.current_q: Optional[np.ndarray] = None
        self.current_tcp_position: Optional[np.ndarray] = None
        self.current_tcp_rotation: Optional[np.ndarray] = None

        # =========================
        # ROS
        # =========================
        self.joint_state_sub = self.create_subscription(
            JointState, "/joint_states", self.joint_state_callback, 10
        )

        self.tcp_pose_sub = self.create_subscription(
            PoseStamped, "/tcp_pose_broadcaster/pose", self.tcp_pose_callback, 10
        )

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, self.command_topic, 10
        )

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.control_loop)

        self.get_logger().info(f"Nó iniciado. Publicando em: {self.command_topic}")
        self.get_logger().info(
            f"Alvo posição: x={self.target_position[0]:.3f}, "
            f"y={self.target_position[1]:.3f}, z={self.target_position[2]:.3f}"
        )
        self.get_logger().info(
            f"kp_pos={self.kp_pos:.3f}, kp_ori={self.kp_ori:.3f}, "
            f"damping={self.damping:.3f}, max_velocity={self.max_velocity:.3f}"
        )

    # =========================================================
    # CALLBACKS
    # =========================================================
    def joint_state_callback(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))

        try:
            ordered_q = np.array(
                [float(name_to_pos[name]) for name in self.joint_names_expected],
                dtype=float,
            )
            self.current_q = ordered_q
        except KeyError:
            missing = [
                name for name in self.joint_names_expected if name not in name_to_pos
            ]
            self.get_logger().warn(
                f"Joint states incompletos. Faltando: {missing}",
                throttle_duration_sec=2.0,
            )

    def tcp_pose_callback(self, msg: PoseStamped) -> None:
        self.current_tcp_position = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z],
            dtype=float,
        )

        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        self.current_tcp_rotation = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)

    # =========================================================
    # CINEMÁTICA
    # =========================================================
    def dh_transform(self, a: float, d: float, alpha: float, theta: float) -> np.ndarray:
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ],
            dtype=float,
        )

    def ur3e_dh(self, q: np.ndarray) -> List[Tuple[float, float, float, float]]:
        return [
            (0.0, 0.15185, math.pi / 2.0, q[0]),
            (-0.24355, 0.0, 0.0, q[1]),
            (-0.21320, 0.0, 0.0, q[2]),
            (0.0, 0.13105, math.pi / 2.0, q[3]),
            (0.0, 0.08535, -math.pi / 2.0, q[4]),
            (0.0, 0.09210, 0.0, q[5]),
        ]

    def forward_kinematics_chain(self, q: np.ndarray) -> List[np.ndarray]:
        chain = [np.eye(4)]
        T = np.eye(4)
        for a, d, alpha, theta in self.ur3e_dh(q):
            T = T @ self.dh_transform(a, d, alpha, theta)
            chain.append(T.copy())
        return chain

    def geometric_jacobian(self, q: np.ndarray) -> np.ndarray:
        chain = self.forward_kinematics_chain(q)
        p_tcp_fk = chain[-1][0:3, 3]

        J = np.zeros((6, 6), dtype=float)

        for i in range(6):
            T_i = chain[i]
            z_i = T_i[0:3, 2]
            p_i = T_i[0:3, 3]

            J[0:3, i] = np.cross(z_i, p_tcp_fk - p_i)
            J[3:6, i] = z_i

        return J

    # =========================================================
    # ROTAÇÕES
    # =========================================================
    def rot_x(self, angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)

    def rot_y(self, angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)

    def rot_z(self, angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)

    def rpy_to_rotation_matrix(self, roll: float, pitch: float, yaw: float) -> np.ndarray:
        return self.rot_z(yaw) @ self.rot_y(pitch) @ self.rot_x(roll)

    def quaternion_to_rotation_matrix(
        self, qx: float, qy: float, qz: float, qw: float
    ) -> np.ndarray:
        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm < 1e-12:
            return np.eye(3)

        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm

        return np.array(
            [
                [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
                [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
                [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
            ],
            dtype=float,
        )

    def orientation_error(self, R_current: np.ndarray, R_desired: np.ndarray) -> np.ndarray:
        return 0.5 * (
            np.cross(R_current[:, 0], R_desired[:, 0]) +
            np.cross(R_current[:, 1], R_desired[:, 1]) +
            np.cross(R_current[:, 2], R_desired[:, 2])
        )

    # =========================================================
    # UTILITÁRIOS
    # =========================================================
    def build_test_command(self) -> np.ndarray:
        cmd = np.zeros(6, dtype=float)
        if 0 <= self.test_joint < 6:
            cmd[self.test_joint] = self.test_velocity
        return cmd

    def damped_least_squares(self, J: np.ndarray, v: np.ndarray, damping: float) -> np.ndarray:
        lambda2 = damping ** 2
        A = J @ J.T + lambda2 * np.eye(J.shape[0])
        return J.T @ np.linalg.solve(A, v)

    def saturate_vector(self, vec: np.ndarray, max_abs_value: float) -> np.ndarray:
        return np.clip(vec, -max_abs_value, max_abs_value)

    def publish_command(self, qdot_cmd: np.ndarray) -> None:
        msg = Float64MultiArray()
        msg.data = qdot_cmd.tolist()
        self.cmd_pub.publish(msg)

    def stop_robot(self) -> None:
        self.publish_command(np.zeros(6, dtype=float))

    # =========================================================
    # LOOP
    # =========================================================
    def control_loop(self) -> None:
        if self.current_q is None:
            self.get_logger().warn(
                "Aguardando /joint_states...",
                throttle_duration_sec=2.0,
            )
            return

        if self.current_tcp_position is None or self.current_tcp_rotation is None:
            self.get_logger().warn(
                "Aguardando /tcp_pose_broadcaster/pose...",
                throttle_duration_sec=2.0,
            )
            return

        if self.use_joint_test:
            self.publish_command(self.build_test_command())
            return

        p_measured = self.current_tcp_position
        R_measured = self.current_tcp_rotation

        e_p = self.target_position - p_measured
        e_o = self.orientation_error(R_measured, self.target_rotation)

        pos_error_norm = float(np.linalg.norm(e_p))
        ori_error_norm = float(np.linalg.norm(e_o))

        self.get_logger().info(
            f"TCP real: x={p_measured[0]:.4f}, y={p_measured[1]:.4f}, z={p_measured[2]:.4f} | "
            f"ep={pos_error_norm:.4f} m | eo={ori_error_norm:.4f}",
            throttle_duration_sec=1.0,
        )

        if pos_error_norm < self.pos_tolerance and ori_error_norm < self.ori_tolerance:
            self.publish_command(np.zeros(6, dtype=float))
            return

        v_linear = self.kp_pos * e_p
        v_angular = self.kp_ori * e_o
        v = np.hstack((v_linear, v_angular))

        J = self.geometric_jacobian(self.current_q)
        qdot_cmd = self.damped_least_squares(J, v, self.damping)
        qdot_cmd = self.saturate_vector(qdot_cmd, self.max_velocity)

        self.publish_command(qdot_cmd)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointVelocityCtrl()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Encerrando nó. Enviando comando zero.")
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()