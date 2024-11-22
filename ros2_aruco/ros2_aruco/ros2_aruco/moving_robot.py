import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


flag = True

class TwistPublisherNode(Node):
    def __init__(self):
        super().__init__('twist_publisher')
        
        # Crea il publisher per il topic /cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)  # 10 è la coda dei messaggi
        
        # Imposta la frequenza desiderata in Hz
        self.frequency = 10.0  # 10 Hz
        self.timer_period = 1.0 / self.frequency  # Periodo del timer in secondi (0.1 secondi per 10 Hz)
        self.subscription = self.create_subscription(
            Bool,
            '/stop_flag',
            self.listener_callback,
            10
        )
        
        # Crea il timer per chiamare il callback alla frequenza desiderata
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    def listener_callback(self, msg):
            global flag
            self.get_logger().info(f'Received: {msg.data}')
            if msg.data:
                flag=True
            else:
                flag=False
    
    def timer_callback(self):
        # Crea e pubblica il messaggio Twist
        twist_msg = Twist()
        if flag: #stop flag is true?
            twist_msg.linear.x  = 0.0
            twist_msg.linear.y  = 0.0
            twist_msg.linear.z  = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0  # Ruota sull'asse Z
        else:
            twist_msg.linear.x  = 0.0
            twist_msg.linear.y  = 0.0
            twist_msg.linear.z  = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.3  # Ruota sull'asse Z
        # Pubblica il messaggio
        self.publisher_.publish(twist_msg)
        #self.get_logger().info(f'Published: {twist_msg}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

