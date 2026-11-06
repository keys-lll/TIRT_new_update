"""wp_record: drive the car into place, type a name, and its pose is saved."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from visualization_msgs.msg import MarkerArray

from car_waypoints.ros_msgs import build_markers
from car_waypoints.shell import RecordShell
from car_waypoints.store import DEFAULT_FILE, load_waypoints
from car_waypoints.tf_pose import init_ros, lookup_pose, make_listener, stop_listener


class Recorder(Node):
    def __init__(self):
        super().__init__('wp_record')
        self.waypoint_file = self.declare_parameter('file', DEFAULT_FILE).value
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.buffer, self.listener = make_listener()
        # transient_local：RViz 比這個程式晚開也看得到框
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.marker_pub = self.create_publisher(MarkerArray, 'waypoint_markers', latched)

    def current_pose(self):
        return lookup_pose(self.buffer, self.map_frame, self.base_frame)

    def close(self):
        stop_listener(self.listener)
        self.destroy_node()

    def publish_markers(self, waypoints):
        self.marker_pub.publish(build_markers(waypoints, self.map_frame))


def main():
    init_ros()
    node = Recorder()
    shell = RecordShell(node.waypoint_file, node.current_pose, on_change=node.publish_markers)
    try:
        node.publish_markers(load_waypoints(node.waypoint_file))
    except ValueError as e:
        print(f'點位檔有錯：{e}')
    print(f'點位檔：{node.waypoint_file}')
    print('建議存點：start、prep、t1～t5、a（備餐區前方）、b（5 號桌旁）')
    print('A／a 存為 a；B／b 存為 b。A、B 記錄時不必刻意對準車頭。')
    print('把車開到定位，輸入名稱就存；help 看說明，q 離開。RViz 加 MarkerArray 看 /waypoint_markers')
    try:
        running = True
        while running:
            try:
                text, running = shell.handle(input('wp_record> '))
            except EOFError:
                break
            if text:
                print(text)
    except KeyboardInterrupt:
        print()
    finally:
        node.close()
        rclpy.try_shutdown()
