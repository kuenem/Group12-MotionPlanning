import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
import signal
import sys

class TFJointStateSaver(Node):
    def __init__(self):
        super().__init__('tf_jointstate_saver')
        self.latest_tf = None
        self.latest_joint_states = None

        self.tf_sub = self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)

    def tf_callback(self, msg):
        self.latest_tf = msg
        # print('recieved a /tf message', msg)

    def joint_states_callback(self, msg):
        self.latest_joint_states = msg
        # print('recieved a /joint_states message', msg)

    def save_to_file(self):
        with open('latest_ros2_messages.txt', 'w') as f:
            f.write("Latest /tf message:\n")
            if self.latest_tf:
                for transform in self.latest_tf.transforms:
                    f.write(f"  Frame: {transform.header.frame_id} -> {transform.child_frame_id}\n")
                    t = transform.transform.translation
                    r = transform.transform.rotation
                    f.write(f"    Translation: x={t.x}, y={t.y}, z={t.z}\n")
                    f.write(f"    Rotation: x={r.x}, y={r.y}, z={r.z}, w={r.w}\n")
            else:
                f.write("  No /tf message received yet.\n")

            f.write("\nLatest /joint_states message:\n")
            if self.latest_joint_states:
                f.write(f"  Names: {self.latest_joint_states.name}\n")
                f.write(f"  Positions: {self.latest_joint_states.position}\n")
                f.write(f"  Velocities: {self.latest_joint_states.velocity}\n")
                f.write(f"  Efforts: {self.latest_joint_states.effort}\n")
            else:
                f.write("  No /joint_states message received yet.\n")

        self.get_logger().info('Messages saved to latest_ros2_messages.txt')

def main(args=None):
    rclpy.init(args=args)
    node = TFJointStateSaver()

    def shutdown_handler(signum, frame):
        node.save_to_file()
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    rclpy.spin(node)
    

if __name__ == '__main__':
    main()
