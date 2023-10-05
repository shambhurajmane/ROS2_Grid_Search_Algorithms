# This node subscribes to "custom_occupancy_grid" topic to get the grid occupancy status and uses the data to calculate the path from Northwest
# corner to southeast corner. Also it publishes the path on as a points and connecting lines on the "dijkstra_topic" and publishes the no of iterations 
# on the "dijkstra_iterations" topic. 


import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Int32MultiArray
from visualization_msgs.msg import Marker
import geometry_msgs.msg 
import heapq


class DJ_publisher(Node):

    def __init__(self):
        super().__init__('dj_publisher')
        self.grid=[]
        self.start_x, self.start_y = 0,0
        self.dest_x, self.dest_y = 127, 127
        self.dx = [-1, 1, 0, 0]
        self.dy = [0, 0, -1, 1]
        self.visited=[]
        self.width=0
        self.visited_iterations=0
        self.path_length=0
        self.occupancy_array=[]

        self.subscription = self.create_subscription(OccupancyGrid, 'custom_occupancy_grid', self.grid_callback, 10) 

        self.dj_publisher = self.create_publisher(Marker, 'dijkstra_topic', 10)
        self.iteration_publisher = self.create_publisher(Int32MultiArray, 'dijkstra_iterations', 10)
        self.grid_publisher = self.create_publisher(OccupancyGrid, 'custom_occupancy_grid', 10)
                 
    
    def grid_callback(self,grid_message):
        self.occupancy_array= grid_message.data
        self.width=grid_message.info.width
        pos=[]
        for i in range(0,len(self.occupancy_array)):
            
            if len(pos)<self.width:
                pos.append(self.occupancy_array[i])

            else:
                self.grid.append(pos)
                pos=[]
                pos.append(self.occupancy_array[i])
        self.grid.append(pos) 
        self.dijkstra()
        self.deploy_marker()
        #print(self.grid)


    def is_valid(self,x, y):
        #print(x,y,grid)
        if 0 <= x < len(self.grid) and 0 <= y < len(self.grid[0]):
            return True
        return False

    def dijkstra(self):
        heap = [(0, (self.start_x, self.start_y))]
        distance = {(i, j): float('inf') for i in range(self.width) for j in range(self.width)}
        distance[(self.start_x, self.start_y)] = 0
        parent = {}

        while heap:
            self.visited_iterations+=1
            dist, (x, y) = heapq.heappop(heap)
            #print(x,y)

            if x == self.dest_x and y == self.dest_y:
                path = []
                while (x, y) in parent:
                    path.append((x, y))
                    x, y = parent[(x, y)]

                path.append((self.start_x, self.start_y))
                
                #print("path found and publsihing on topic")
                self.visited=path
                self.path_length=len(path)
                return self.visited    
            
            if dist > distance[(x, y)]:
                continue

            for i in range(4):
                new_x, new_y = x + self.dx[i], y + self.dy[i]
                if self.is_valid(new_x, new_y):
                    if self.grid[new_x][new_y]==0:
                        self.occupancy_array[(new_x*self.width)+new_y]=50
                        self.grid_pub()
                        new_dist = distance[(x, y)] + 1
                        if new_dist < distance[(new_x, new_y)]:
                            distance[(new_x, new_y)] = new_dist
                            parent[(new_x, new_y)] = (x, y)
                            heapq.heappush(heap, (new_dist, (new_x, new_y)))
                            

        #print("path not found")
        return self.visited
    
    

    def deploy_marker(self):
        
        line_strip=Marker()
        points=Marker()
        array=Int32MultiArray()
        #print(self.final_path)
        
        line_strip.header.frame_id = "/map_frame"
        line_strip.header.stamp = DJ_publisher.get_clock(self).now().to_msg()
        points.header.frame_id = "/map_frame"
        points.header.stamp = DJ_publisher.get_clock(self).now().to_msg()

        line_strip.ns = points.ns ="lines"
        points.id=0
        line_strip.id = 1
        line_strip.type = Marker.LINE_STRIP
        line_strip.action = points.action = Marker.ADD
        line_strip.pose.position.x = -4.5
        line_strip.pose.position.y = -4.5
        line_strip.pose.position.z = 0.0

        line_strip.pose.orientation.w = 1.0

        #for 10*10 grid make scale as 0.1 and for 128*128 make it as 0.8
        line_strip.scale.x = 0.8

        line_strip.color.g = 1.0
        line_strip.color.a = 1.0

        points.type = Marker.POINTS
     
        points.pose.position.x = -4.5
        points.pose.position.y = -4.5
        points.pose.position.z = 0.0
        points.pose.orientation.w = 1.0
        points.scale.x = 0.4
        points.scale.y = 0.4
        #marker.scale.z = 0.1

        points.color.b = 1.0
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
        self.dj_publisher.publish(line_strip)
        self.dj_publisher.publish(points)

    def grid_pub(self):
        message=OccupancyGrid()
        
        message.header.stamp = DJ_publisher.get_clock(self).now().to_msg()
        message.header.frame_id = "map_frame"
        message.info.resolution = 1.0
        message.info.width = self.width
        message.info.height = self.width
        message.info.origin.position.x = -5.0
        message.info.origin.position.y = -5.0
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.x = 0.0
        message.info.origin.orientation.y = 0.0
        message.info.origin.orientation.z = 0.0
        message.info.origin.orientation.w = 1.0
        message.data = self.occupancy_array

        self.grid_publisher.publish(message)

def main(args=None):
    rclpy.init(args=args)
    dj_publisher = DJ_publisher()
    rclpy.spin_once(dj_publisher)
    dj_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()