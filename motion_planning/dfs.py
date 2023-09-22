# This node subscribes to "custom_occupancy_grid" topic to get the grid occupancy status and uses the data to calculate the path from Northwest
# corner to southeast corner. Also it publishes the path on as a points and connecting lines on the "dfs_topic" and publishes the no of iterations 
# on the "dfs_iterations" topic. 


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

from collections import deque
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker
import geometry_msgs.msg 


class DFSPublisher(Node):

    def __init__(self):
        super().__init__('dfs_publisher')

        #Change values here to input new start and destination points
        self.grid=[]
        self.start_x, self.start_y = 0,0
        self.dest_x, self.dest_y = 127, 127
        self.dx = [0, 1, -1, 0]
        self.dy = [1, 0, 0, 1]
        self.visited=[]
        self.visited_iterations=0
        self.path_length=0

        self.subscription = self.create_subscription(OccupancyGrid, 'custom_occupancy_grid', self.grid_callback, 10) 

        self.dfs_publisher = self.create_publisher(Marker, 'dfs_topic', 10)
        self.iteration_publisher = self.create_publisher(Int32MultiArray, 'dfs_iterations', 10)
        

        
                 
    
    def grid_callback(self,grid_message):
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
        self.dfs()
        self.deploy_marker()
        #print(self.grid)


    def is_valid(self,x, y):
        #print(x,y,grid)
        if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
            return True
        return False
    
    def dfs(self):

        def dfs_in(x,y,path):
            if x == self.dest_x and y == self.dest_y:
                #print("path found")
                return path + [(x, y)]

            self.visited.append((x, y))

            for i in range(4):
                new_x, new_y = x + self.dx[i], y + self.dy[i]
                self.visited_iterations+=1
                if self.is_valid(new_x, new_y) and (new_x, new_y) not in self.visited:
                    if self.grid[new_x][new_y]==0:
                        new_path = dfs_in(new_x, new_y, path + [(x, y)])
                        if new_path:
                            return new_path

            return []

        path = dfs_in(self.start_x, self.start_y, [])
        self.visited=path
        #print(self.grid)
        self.path_length=len(path)
        return self.visited
    
    

    def deploy_marker(self):
        
        line_strip=Marker()
        points=Marker()
        array=Int32MultiArray()
        #print(self.final_path)
        
        line_strip.header.frame_id = "/map_frame"
        line_strip.header.stamp = DFSPublisher.get_clock(self).now().to_msg()
        points.header.frame_id = "/map_frame"
        points.header.stamp = DFSPublisher.get_clock(self).now().to_msg()

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
        line_strip.scale.x = 0.8

        line_strip.color.b = 1.0
        line_strip.color.a = 1.0

        points.type = Marker.POINTS
     
        points.pose.position.x = -4.5
        points.pose.position.y = -4.5
        points.pose.position.z = 0.0
        points.pose.orientation.w = 1.0

        #for 10*10 grid make scale as 0.1 and for 128*128 make it as 1.0
        points.scale.x = 0.4
        points.scale.y = 0.4

        points.color.r = 1.0
        points.color.a = 1.0

        p=geometry_msgs.msg.Point()
        for i in range(0,len(self.visited)):
            p.x=self.visited[i][0]*1.0
            p.y=self.visited[i][1]*1.0
            p.z=0.0
            #print(p.x,p.y)
            points.points.append(geometry_msgs.msg.Point(x=self.visited[i][1]*1.0, y=self.visited[i][0]*1.0, z=0.0))
            line_strip.points.append(geometry_msgs.msg.Point(x=self.visited[i][1]*1.0, y=self.visited[i][0]*1.0, z=0.0))
        #print(line_strip.points)

        array.data=[self.visited_iterations,self.path_length]
        self.iteration_publisher.publish(array)
        self.dfs_publisher.publish(line_strip)
        self.dfs_publisher.publish(points)

def main(args=None):
    rclpy.init(args=args)
    dfs_publisher = DFSPublisher()
    rclpy.spin_once(dfs_publisher)
    dfs_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()