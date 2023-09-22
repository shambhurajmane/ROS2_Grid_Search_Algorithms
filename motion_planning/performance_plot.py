# This node have subscriber for all search algorithms to get the no of iteration they took for calculating th path.
# It plots the graph of obstacle percentage vs all search algorithms ie bfs, dfs and dijkstra, when 10 entriws are recieved.



import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
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
        # x-axis label
        plt.xlabel('Percentage')
        # frequency label
        plt.ylabel('No. of iterations')
        # plot title
        plt.title('Motion Planning: Flatland assignemnt')

        self.subscription = self.create_subscription(Int32MultiArray, 'bfs_iterations', self.bfs_callback, 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'dfs_iterations', self.dfs_callback, 10)
        self.subscription = self.create_subscription(Int32MultiArray, 'dijkstra_iterations', self.dijsktra_callback, 10) 
        self.subscription = self.create_subscription(Float32, 'obstacle_percentage', self.percent_callback, 10) 
        self.subscription = self.create_subscription(Int32MultiArray, 'random_iterations', self.random_callback, 10) 
     
        
                 
    def percent_callback(self, message):
        percentage=message.data
        
        self.per_data.append(int(percentage))
        print(self.per_data)


    def bfs_callback(self, message):
        value=message.data[0]
        self.bfs_data.append(value)
        print("bfs")
        print(self.bfs_data)
        
        if len(self.bfs_data)==len(self.dfs_data)==len(self.dijkstra_data)==len(self.random_data)==6:
            plt.plot(self.per_data, self.dfs_data, label = "dfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.bfs_data, label = "bfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.dijkstra_data, label = "dijkstra", marker='o', markersize=12)
            plt.plot(self.per_data, self.random_data, label = "random", marker='o', markersize=12)
            plt.legend()
            plt.show()

    def dfs_callback(self, message):
        value=message.data[0]
        self.dfs_data.append(value)
        print("dfs")
        print(self.dfs_data)
        if len(self.bfs_data)==len(self.dfs_data)==len(self.dijkstra_data)==len(self.random_data)==6:
            plt.plot(self.per_data, self.dfs_data, label = "dfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.bfs_data, label = "bfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.dijkstra_data, label = "dijkstra", marker='o', markersize=12)
            plt.plot(self.per_data, self.random_data, label = "random", marker='o', markersize=12)
            plt.legend()
            plt.show()

    def dijsktra_callback(self, message):
        value=message.data[0]
        self.dijkstra_data.append(value)
        print("dijkstra")
        print(self.dijkstra_data)
        if len(self.bfs_data)==len(self.dfs_data)==len(self.dijkstra_data)==len(self.random_data)==6:
            plt.plot(self.per_data, self.dfs_data, label = "dfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.bfs_data, label = "bfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.dijkstra_data, label = "dijkstra", marker='o', markersize=12)
            plt.plot(self.per_data, self.random_data, label = "random", marker='o', markersize=12)
            plt.legend()
            plt.show()

    def random_callback(self, message):
        value=message.data[0]
        self.random_data.append(value)
        print("random")
        print(self.random_data)
        if len(self.bfs_data)==len(self.dfs_data)==len(self.dijkstra_data)==len(self.random_data)==6:
            plt.plot(self.per_data, self.dfs_data, label = "dfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.bfs_data, label = "bfs", marker='o', markersize=12)
            plt.plot(self.per_data, self.dijkstra_data, label = "dijkstra", marker='o', markersize=12)
            plt.plot(self.per_data, self.random_data, label = "random", marker='o', markersize=12)
            plt.legend()
            plt.show()
     

def main(args=None):
    rclpy.init(args=args)
    plot_publisher = GridPublisher()
    rclpy.spin(plot_publisher)
    plot_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()