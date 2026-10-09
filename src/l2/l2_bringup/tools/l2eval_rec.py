# l2eval_rec.py OUT.csv : log /pcl_pose and /aft_mapped_to_init as "tag,stamp,x,y,z,qx,qy,qz,qw"
import sys, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
f = open(sys.argv[1], 'w')
def w(tag, h, p):
    f.write(f"{tag},{h.stamp.sec + h.stamp.nanosec*1e-9:.6f},{p.position.x},{p.position.y},{p.position.z},"
            f"{p.orientation.x},{p.orientation.y},{p.orientation.z},{p.orientation.w}\n"); f.flush()
rclpy.init(); n = Node('rec')
q = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
n.create_subscription(Odometry, '/aft_mapped_to_init', lambda m: w('lio', m.header, m.pose.pose), 1000)
n.create_subscription(PoseWithCovarianceStamped, '/pcl_pose', lambda m: w('loc', m.header, m.pose.pose), q)
try: rclpy.spin(n)
except KeyboardInterrupt: pass
