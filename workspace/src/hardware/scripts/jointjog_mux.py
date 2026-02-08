import rclpy
from rclpy.node import Node

from control_msgs.msg import JointJog
from hardware.msg import JointControl

class JointJogMux(Node):
    def __init__(self):
        super().__init__('jointjog_mux')
        self.declare_parameter('in_topic', 'joint_jog')
        self.declare_parameter('out_topic', 'hardware_node/joint_command')
        self.declare_parameter('stream_topic', 'hardware_node/joint_jog_stream')

        in_topic = self.get_parameter('in_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('out_topic').get_parameter_value().string_value
        stream_topic = self.get_parameter('stream_topic').get_parameter_value().string_value

        # Publisher for converted custom message
        self.pub_converted = self.create_publisher(JointControl, out_topic, 10)
        # Publisher to stream original JointJog messages (for monitoring)
        self.pub_stream = self.create_publisher(JointJog, stream_topic, 10)

        self.sub = self.create_subscription(JointJog, in_topic, self.callback, 10)
        self.get_logger().info(f'JointJogMux listening on {in_topic}, publishing {out_topic} and streaming to {stream_topic}')

    def callback(self, msg: JointJog):
        # Republish the original message on the stream topic
        try:
            self.pub_stream.publish(msg)
        except Exception:
            # Non-fatal; continue
            pass

        out = JointControl()
        out.header = msg.header

        # JointJog message fields vary among ROS versions: try common names
        names = getattr(msg, 'joint_names', [])
        # possible field names for position-like values
        positions = None
        velocities = None
        if hasattr(msg, 'displacements'):
            positions = list(getattr(msg, 'displacements'))
        elif hasattr(msg, 'deltas'):
            positions = list(getattr(msg, 'deltas'))
        elif hasattr(msg, 'delta'):
            positions = list(getattr(msg, 'delta'))

        if hasattr(msg, 'velocities'):
            velocities = list(getattr(msg, 'velocities'))
        elif hasattr(msg, 'speeds'):
            velocities = list(getattr(msg, 'speeds'))

        # Fallbacks: if the message doesn't have positions, use zeros of same size
        n = len(names)
        if positions is None:
            positions = [0.0] * n
        if velocities is None:
            velocities = [0.0] * n

        # Trim or extend to match joint_names length
        positions = (positions + [0.0] * n)[:n]
        velocities = (velocities + [0.0] * n)[:n]

        out.joint_names = list(names)
        out.position = [float(p) for p in positions]
        out.velocity = [float(v) for v in velocities]
        out.effort = [0.0] * n

        self.pub_converted.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = JointJogMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
