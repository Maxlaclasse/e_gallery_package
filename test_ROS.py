import asyncio
import nest_asyncio
import websockets
import struct
import rospy
from sensor_msgs.msg import LaserScan
import math

rospy.init_node('laser_scan_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 100
laser_frequency = 40

nest_asyncio.apply()

POINT_PER_PACK = 12

def sendROS(frame):
    lidar_frame = bytearray(frame)
       

    format_string = "<B B H H H B H B H B H B H B H B H B H B H B H B H B H B H H B"

    # Assume the data is stored in a bytearray named "data"
    frame = struct.unpack(format_string, lidar_frame)

    points = [(frame[i], frame[i+1]) for i in range(4, 4 + POINT_PER_PACK * 2, 2)]
    start_angle = frame[3]/100
    end_angle = frame[-3]/100

    if(end_angle-start_angle > 0):
           step_angle = (end_angle-start_angle)/11
    else:
           step_angle = (end_angle+360-start_angle)/11
    if(step_angle <= 4):         
           
    
        current_time = rospy.Time.now()
        scan = LaserScan()
        
        scan.header.stamp = current_time
        scan.header.frame_id = 'laser_frame'


        scan.angle_min = (start_angle)*math.pi/180
        scan.angle_max = (end_angle)*math.pi/180

        scan.angle_increment = float(step_angle)*math.pi/180

        scan.range_min = 0.0
        scan.range_max = 12.0
        
        scan.ranges = []
        scan.intensities = []
        for i in range(0,12):
            scan.ranges.append(points[i][0]/1000)
            scan.intensities.append(points[i][1])
        
        print(scan)
        scan_pub.publish(scan)

async def recv(websocket):
    
    #r = rospy.Rate(100)
    while not rospy.is_shutdown():
       
        async for message in websocket:
               if rospy.is_shutdown():
                    break
            
               sendROS(message)
               #r.sleep()
        
async def main():
    async with websockets.serve(recv, "172.22.79.29", 8085):
        await asyncio.Future()  # run forever


asyncio.run(main())
