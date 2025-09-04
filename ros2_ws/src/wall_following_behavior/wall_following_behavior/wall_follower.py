import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class SimplePublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('wall_follower_node')

        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the listener_callback method.
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data and some sensor data.
        self.left_ray = 0.3
        self.front_ray = 0.5

        # define the timer period for 0.5 seconds
        timer_period = 0.5
        # create a timer sending two parameters:
        # - the duration between 2 callbacks (0.5 seconds)
        # - the timer function (timer_callback)
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def laser_callback(self, msg):
        # print the log info in the terminal
        #self.left_ray = msg.ranges[37]
        #self.front_ray = msg.ranges[20]
        self.left_ray = msg.ranges[209]
        self.front_ray = msg.ranges[115]

    def timer_callback(self):
        # Here you have the callback method
        # create a Twist message
        msg = Twist()

        self.get_logger().info('current left_ray: "%f"' % self.left_ray)
        self.get_logger().info('current front_ray: "%f"' % self.front_ray)


        if self.front_ray < 0.45:
            self.get_logger().info("Turn fast to right")
            msg.linear.x = 0.025
            msg.angular.z = -0.5
        elif self.left_ray > 0.255:
            self.get_logger().info("Find the wall")
            msg.linear.x = 0.05
            msg.angular.z = 0.05
        elif self.left_ray < 0.2:
            self.get_logger().info("Move away from the wall")
            msg.linear.x = 0.025
            msg.angular.z = -0.25
        elif self.left_ray > 0.2 and self.left_ray < 0.3:
            self.get_logger().info("Follow the wall")
            msg.linear.x = 0.05
            msg.angular.z = 0.0
        else:
            self.get_logger().info("Stop")
            msg.linear.x = 0.0
            msg.angular.z = 0.0
       
        # Publish the message to the Topic
        self.publisher_.publish(msg)
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_publisher = SimplePublisher()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroys the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()