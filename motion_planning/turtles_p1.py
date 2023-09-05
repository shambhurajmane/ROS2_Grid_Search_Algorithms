from rclpy.node import Node
import rclpy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_srvs.srv import Empty


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('MinimalPublisher') 

        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10) 
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel',10) 
        
    def pose_callback(self,pose_message):
        global x
        global y, yaw
        x = pose_message.x
        y = pose_message.y
        yaw = pose_message.theta
        self.move1(self.publisher, 1.0, 5.0)

    def move1(self,velocity_publisher, speed, distance):
        self.start_time = self.get_clock().now() 
        
        velocity_message = Twist()
        velocity_message.linear.x = abs(speed)
        
        distance_moved = 0.0
        
        while (distance_moved<=distance):
            velocity_publisher.publish(velocity_message)
            
            distance_moved = speed * (self.get_clock().now().nanoseconds - self.start_time.nanoseconds)/1e9 
            print(distance_moved)

        
        velocity_message.linear.x = 0.0
        velocity_publisher.publish(velocity_message)
        self.rotate1(self.publisher, 1.0, 2.094)

    def rotate1(self,velocity_publisher, speed, angle):
        self.start_time = self.get_clock().now() 
        
        velocity_message = Twist()
        velocity_message.angular.z = - abs(speed)
        
        angle_moved = 0.0
        
        while (angle_moved<=angle):
            velocity_publisher.publish(velocity_message)
            
            angle_moved = speed * (self.get_clock().now().nanoseconds - self.start_time.nanoseconds)/1e9 
            print(angle_moved)

        
        
        velocity_message.angular.z = 0.0
        velocity_publisher.publish(velocity_message)
        self.move2(self.publisher, 1.0, 5.0)


    def move2(self,velocity_publisher, speed, distance):
        self.start_time = self.get_clock().now() 
        
        velocity_message = Twist()
        velocity_message.linear.x = abs(speed)
        
        distance_moved = 0.0
        
        while (distance_moved<=distance):
            velocity_publisher.publish(velocity_message)
            
            distance_moved = speed * (self.get_clock().now().nanoseconds - self.start_time.nanoseconds)/1e9 
            print(distance_moved)

        
        
        velocity_message.linear.x = 0.0
        velocity_publisher.publish(velocity_message)
        self.rotate2(self.publisher, 1.0, 2.094)


    def rotate2(self,velocity_publisher, speed, angle):
        self.start_time = self.get_clock().now() 
        
        velocity_message = Twist()
        velocity_message.angular.z = - abs(speed)
        
        angle_moved = 0.0
        
        while (angle_moved<=angle):
            velocity_publisher.publish(velocity_message)
            
            angle_moved = speed * (self.get_clock().now().nanoseconds - self.start_time.nanoseconds)/1e9 
            print(angle_moved)

        velocity_message.angular.z = 0.0
        velocity_publisher.publish(velocity_message)
        self.move1(self.publisher, 1.0, 10.0)

def main(args=None):
    rclpy.init(args=args) 

    minimal_publisher = MinimalPublisher() 
    rclpy.spin(minimal_publisher) 
    minimal_publisher.move(minimal_publisher.publisher, 2.0, 5.0, True)

    

    minimal_publisher.destroy_node() 
    rclpy.shutdown() 


if __name__ == "__main__":
    main()