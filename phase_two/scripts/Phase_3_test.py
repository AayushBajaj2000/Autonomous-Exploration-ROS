#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose

count = 0

# Just trying to see if we can change the map or not we can but not sure if we should here.
def callback(data):
    length = len(data.data)
    print(length)

    new_costmap = data
    arr = []
    for x in range(0, length):
        arr.append(100)
    
    new_costmap.data = tuple(arr)

    m_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        connections = m_pub.get_num_connections()
        if connections > 0:
            m_pub.publish(new_costmap)
            print("Published map")
            break
        rate.sleep()

def main():
    rospy.init_node('testing', anonymous=True)
    rospy.Subscriber('/map', OccupancyGrid , callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
