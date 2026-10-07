import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Twist

class CmdVelConverter(Node):
    """/cmd_velに送られてきたTwistを、ステアリング角と前後の速度に変換し、シリアル経由でArduinoに送るノード"""

    STEERING_ANGLE_MIN = -10
    STEERING_ANGLE_MAX = 10
    STEERING_SERVO_MIN = 40
    STEERING_SERVO_MAX = 130
    STEERING_SERVO_CTR = 85

    THROTTLE_PWM_MIN = 0
    THROTTLE_PWM_MAX = 100

    def __init__(self):
        super().__init__('cmd_vel_converter')
        
        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cv,
            10
        )

    def cv(self, msg):
        """
        Twistのフィールド
        linear.x : 前後の速度 [m/s]
        angular.z: 旋回の角速度 [rad/s]

        旋回半径R、ホイールベースL、前輪の舵角delta
        R = v / omg
        tan(delta) = L / R
        """
