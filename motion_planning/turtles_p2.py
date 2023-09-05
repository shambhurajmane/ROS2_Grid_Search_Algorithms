import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import random


class GridPublisher(Node):

    def __init__(self):
        super().__init__('grid_publisher')
        #Change values here to get different grid dimensions
        self.width=128              
        self.height=128
        #Change bstacle coverage percentage here to get different grid occupancies
        percent=0.70

        self.occupancy_array=[0]*(self.width*self.height)
        self.total_tenderion=int((self.width*self.height)*percent/4)

        self.grid_publisher = self.create_publisher(OccupancyGrid, 'custom_occupancy_grid', 10)
        self.calculate_grid(self.occupancy_array,self.width,self.height, self.total_tenderion)
                 
    def calculate_grid(self,occupancy_array,width, height, total_tenderion):
        
        for i in range(total_tenderion):
            j=random.randint(1, 4)
            k=random.randint(0,(width*height))
            print(j,k)

            if j==1 and k<(width*(height-3)) and occupancy_array[k]==0 and occupancy_array[k+width]==0 and occupancy_array[k+(2*width)]==0 and occupancy_array[k+(3*width)]==0:
                occupancy_array[k]=100
                occupancy_array[k+(1*width)]=100
                occupancy_array[k+(2*width)]=100
                occupancy_array[k+(3*width)]=100
                print("yes")
 
            elif j==2 and k<(width*(height-2)) and (k%width)>0 and occupancy_array[k]==0 and occupancy_array[k+1]==0 and occupancy_array[k+1+(width)]==0 and occupancy_array[k+1+(2*width)]==0:
                occupancy_array[k]=100
                occupancy_array[k+1]=100
                occupancy_array[k+1+(width)]=100
                occupancy_array[k+1+(2*width)]=100
                print("No")

            elif j==3 and k<(width*(height-2)) and (k%width)>0 and occupancy_array[k]==0 and occupancy_array[k+width]==0 and occupancy_array[k+1+(width)]==0 and occupancy_array[k+1+(2*width)]==0:
                occupancy_array[k]=100
                occupancy_array[k+(1*width)]=100
                occupancy_array[k+1+(1*width)]=100
                occupancy_array[k+1+(2*width)]=100
                print("No")
            
            elif j==4 and k<(width*(height-2)) and (k%width)>0 and occupancy_array[k]==0 and occupancy_array[k+width]==0 and occupancy_array[k+(2*width)]==0 and occupancy_array[k-1+(width)]==0:
                occupancy_array[k]=100
                occupancy_array[k+(1*width)]=100
                occupancy_array[k+(2*width)]=100
                occupancy_array[k-1+(width)]=100
                print("No")
            
            else:
                i=i-1
                print("ohh no")

        timer_period = 0.02 # Execution time
        self.timer = self.create_timer(timer_period, self.grid_callback) 

    def grid_callback(self):
        message=OccupancyGrid()

        message.header.stamp = GridPublisher.get_clock(self).now().to_msg()
        message.header.frame_id = "map_frame"
        message.info.resolution = 10/self.width
        message.info.width = self.width
        message.info.height = self.height
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
    grid_publisher = GridPublisher()
    rclpy.spin(grid_publisher)
    grid_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()