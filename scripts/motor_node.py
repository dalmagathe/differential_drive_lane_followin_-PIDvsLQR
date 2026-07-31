#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Motor param
KT = 0.478   # N.m/A
KE = 0.478   # V.s/rad
R  = 5.0     # Ohm

V_MAX = 5.0  # V


class MotorNode(Node):
    def __init__(self):
        super().__init__('motor_node')

        self.create_subscription(
            Float64MultiArray, '/motor_voltage_cmd', self.voltage_cb, 10)

        self.effort_pub = self.create_publisher(
            Float64MultiArray, '/wheel_effort_controller/commands', 10)

    def publish_effort(self, tau_l, tau_r):
        out = Float64MultiArray()
        out.data = [tau_l, tau_r]
        self.effort_pub.publish(out)

    def voltage_cb(self, msg: Float64MultiArray):
        # Déclenché à chaque commande du controller
        if len(msg.data) != 2:
            self.get_logger().warn(
                'motor_voltage_cmd attendu de taille 2 [V_L, V_R], reçu %d'
                % len(msg.data))
            return

        v_l = max(-V_MAX, min(V_MAX, msg.data[0]))
        v_r = max(-V_MAX, min(V_MAX, msg.data[1]))

        # Quasi-static electrical: V = R.i + Ke.omega  ->  i = (V - Ke.omega)/R
        # Static gain only: i = V/R.
        # The -Ke.omega (back-EMF) term is carried by the joint's ODE damping
        # (b = Kt.Ke/R = 0.0457), integrated at the 1 ms physics step
        i_l = v_l / R
        i_r = v_r / R

        self.publish_effort(KT * i_l, KT * i_r)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()