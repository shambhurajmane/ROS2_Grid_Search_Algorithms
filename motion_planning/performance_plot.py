# This node have subscriber for all search algorithms to get the no of iteration they took for calculating th path.
# It plots the graph of obstacle percentage vs all search algorithms ie bfs, dfs and dijkstra, when 10 entriws are recieved.



import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32
import matplotlib.pyplot as plt



class GridPublisher(Node):

    def __init__(self):
        super().__init__('plot_publisher')
        self.per_data=[]
        self.bfs_data=[]
        self.dfs_data=[]
        self.random_data=[]
        self.dijkstra_data=[]
        self.bfs_time=[]
        self.dfs_time=[]
        self.random_time=[]
        self.dijkstra_time=[]
       

        self.subscription = self.create_subscription(Float32MultiArray, 'bfs_iterations', self.bfs_callback, 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'dfs_iterations', self.dfs_callback, 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'dijkstra_iterations', self.dijsktra_callback, 10) 
        self.subscription = self.create_subscription(Float32, 'obstacle_percentage', self.percent_callback, 10) 
        self.subscription = self.create_subscription(Float32MultiArray, 'random_iterations', self.random_callback, 10) 
     
        
                 
    def percent_callback(self, message):
        percentage=message.data
        
        self.per_data.append(int(percentage))
        


    def bfs_callback(self, message):
        value=message.data
        self.bfs_time.append(value[1])
        self.bfs_data.append(value[0])
      
        # print(self.bfs_data)
        if len(self.bfs_data)==len(self.dfs_data)==len(self.dijkstra_data)==len(self.random_data)==10:
            self.plot()


    def dfs_callback(self, message):
        value=message.data
        self.dfs_time.append(value[1])
        self.dfs_data.append(value[0])
                             
        if len(self.bfs_time)==len(self.dfs_time)==len(self.dijkstra_time)==len(self.random_time)==10:
            self.plot()


    def dijsktra_callback(self, message):
        value=message.data
        self.dijkstra_data.append(value[0])
        self.dijkstra_time.append(value[1])
        if len(self.bfs_time)==len(self.dfs_time)==len(self.dijkstra_time)==len(self.random_time)==10:
            self.plot()


    def random_callback(self, message):
        value=message.data
        self.random_data.append(value[0])
        self.random_time.append(value[1])
        # print("random")
        # print(self.random_time)
        if len(self.bfs_time)==len(self.dfs_time)==len(self.dijkstra_time)==len(self.random_time)==10:
            self.plot()
            
    def plot(self):     
        # Create box and whisker plots
        print("bfs",self.bfs_time, self.bfs_data )
        print("dfs",self.dfs_time, self.dfs_data )
        print("dij",self.dijkstra_time, self.dijkstra_data )

        plt.figure(figsize=(20, 15))

        # Time Complexity
        plt.subplot(1, 2, 1)
        plt.plot(self.per_data, self.dfs_time, label = "dfs", marker='o', markersize=12)
        plt.plot(self.per_data, self.bfs_time, label = "bfs", marker='o', markersize=12)
        plt.plot(self.per_data, self.dijkstra_time, label = "dijkstra", marker='o', markersize=12)
        # plt.plot(self.per_data, self.random_time, label = "random", marker='o', markersize=12)
        plt.title('Time Complexity')
        plt.ylabel('Cases')
        plt.legend()

        # Space Complexity
        plt.subplot(1, 2, 2)
        plt.plot(self.per_data, self.dfs_data, label = "dfs", marker='o', markersize=12)
        plt.plot(self.per_data, self.bfs_data, label = "bfs", marker='o', markersize=12)
        plt.plot(self.per_data, self.dijkstra_data, label = "dijkstra", marker='o', markersize=12)
        # plt.plot(self.per_data, self.random_data, label = "random", marker='o', markersize=12)
        plt.title('Space Complexity')
        plt.ylabel('Cases')
        plt.legend()
    

        plt.tight_layout()
        plt.show()
   
     

def main(args=None):
    rclpy.init(args=args)
    plot_publisher = GridPublisher()
    rclpy.spin(plot_publisher)
    plot_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()