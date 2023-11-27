# This node subscribes to "custom_occupancy_grid" topic to get the grid occupancy status and uses the data to calculate the path from Northwest
# corner to southeast corner. Also it publishes the path on as a points and connecting lines on the "random_topic" and publishes the no of iterations 
# on the "random_iterations" topic. 


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from collections import deque
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker
import geometry_msgs.msg 
import random
import time



class RandomPublisher(Node):

    def __init__(self):
        super().__init__('random_publisher')

        #Change values here to input new start and destination points
        self.grid=[]
        self.start_x, self.start_y = 0,0
        self.dest_x, self.dest_y = 127, 127
        self.dx = [-1, 1, 0, 0]
        self.dy = [0, 0, -1, 1]
        self.final_path=[]
        self.visited_iterations=0
        self.path_length=0


        self.subscription = self.create_subscription(OccupancyGrid, 'custom_occupancy_grid', self.grid_callback, 10) 
        

        self.random_publisher = self.create_publisher(Marker, 'random_topic', 10)
        self.iteration_publisher = self.create_publisher(Float32MultiArray, 'random_iterations', 10)
        
        

        
           
    
    def grid_callback(self,grid_message):
        self.grid=[]
        self.start_x, self.start_y = 0,0
        self.dest_x, self.dest_y = 127, 127
        self.dx = [-1, 1, 0, 0]
        self.dy = [0, 0, -1, 1]
        self.final_path=[]
        self.visited_iterations=0
        self.path_length=0
        occupancy_array= grid_message.data
        width=grid_message.info.width
        pos=[]
        for i in range(0,len(occupancy_array)):
            
            if len(pos)<width:
                pos.append(occupancy_array[i])

            else:
                self.grid.append(pos)
                pos=[]
                pos.append(occupancy_array[i])
        self.grid.append(pos) 
        
        start = time.time()
        self.random()
        end = time.time()
        time_taken = round((end-start), 4)
        array=Float32MultiArray()
        array.data=[self.visited_iterations * 1.0,time_taken]
        self.iteration_publisher.publish(array)
        self.deploy_marker()
        #print(self.grid)


    def is_valid(self,x, y):
        #print(x,y,grid)
        if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
            return True
        return False

    def random(self):
        x, y = self.start_x, self.start_y
        path = [(x, y)]
        

        while x != self.dest_x or y != self.dest_y:
            if self.visited_iterations>80000:
                # print("path not found by random algorithm ")
                
                return []
            valid_neighbors = []
            for i in range(4):
                self.visited_iterations+=1
                new_x, new_y = x + self.dx[i], y + self.dy[i]
                if self.is_valid(new_x, new_y):
                    if self.grid[new_x][new_y]==0:
                        valid_neighbors.append((new_x, new_y))

            if not valid_neighbors:
                return []

            x, y = random.choice(valid_neighbors)
            path.append((x, y))
        # print("path found by random algorithm with length", len(path))
        self.final_path=path
        return self.final_path
    
    

    def deploy_marker(self):
        
        line_strip=Marker()
        points=Marker()
        array=Float32MultiArray()
        #print(self.final_path)
        
        line_strip.header.frame_id = "/map"
        line_strip.header.stamp = RandomPublisher.get_clock(self).now().to_msg()
        points.header.frame_id = "/map"
        points.header.stamp = RandomPublisher.get_clock(self).now().to_msg()

        line_strip.ns = points.ns ="lines"
        points.id=0
        line_strip.id = 1
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = points.action = Marker.ADD
        line_strip.pose.position.x = -4.5
        line_strip.pose.position.y = -4.5
        line_strip.pose.position.z = 0.0
        line_strip.pose.orientation.w = 1.0

        #for 10*10 grid make scale as 0.1 and for 128*128 make it as 1.0
        line_strip.scale.x = 0.1
        line_strip.color.r = 1.0
        line_strip.color.a = 1.0

        points.type = Marker.POINTS
     
        points.pose.position.x = -4.5
        points.pose.position.y = -4.5
        points.pose.position.z = 0.0
        points.pose.orientation.w = 1.0

        #for 10*10 grid make scale as 0.1 and for 128*128 make it as 1.0
        points.scale.x = 0.1
        points.scale.y = 0.1
 
        points.color.g = 1.0
        points.color.b = 1.0
        points.color.a = 1.0

        p=geometry_msgs.msg.Point()
        for i in range(0,len(self.final_path)):
            p.x=self.final_path[i][0]*1.0
            p.y=self.final_path[i][1]*1.0
            p.z=0.0
            #print(p.x,p.y)
            points.points.append(geometry_msgs.msg.Point(x=self.final_path[i][1]*1.0, y=self.final_path[i][0]*1.0, z=0.0))
            line_strip.points.append(geometry_msgs.msg.Point(x=self.final_path[i][1]*1.0, y=self.final_path[i][0]*1.0, z=0.0))
     
        self.random_publisher.publish(line_strip)
        self.random_publisher.publish(points)

def main(args=None):
    rclpy.init(args=args)
    random_publisher = RandomPublisher()
    rclpy.spin(random_publisher)
    random_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()