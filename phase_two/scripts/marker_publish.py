#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from fiducial_msgs.msg import FiducialTransformArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import tf2_ros
import tf2_geometry_msgs

published = [] 


def transform_pose(input_pose, from_frame, to_frame):
   
    # **Assuming /tf2 topic is being broadcasted
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

def callback(data):
    transformArray = data.transforms

    print("published array: ")
    print(published)

    # Publish it only once -> one reading, maybe we should increase these
    if len(transformArray) > 0 and transformArray[0].fiducial_id not in published:
        transform = transformArray[0]

        m_pub = rospy.Publisher('visualization_marker',  Marker, queue_size=1)
        
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "raspicam"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        marker.ns = "fiducial"
        marker.id =  transform.fiducial_id
	
	# If we want to do it according to the map which we might need for phase 4.
        '''my_pose = Pose()
        my_pose.position.x = transform.transform.translation.x 
        my_pose.position.y = transform.transform.translation.y
        my_pose.position.z = transform.transform.translation.z - 0.5
        my_pose.orientation.x = transform.transform.rotation.x
        my_pose.orientation.y = transform.transform.rotation.y
        my_pose.orientation.z = transform.transform.rotation.z
        my_pose.orientation.w = transform.transform.rotation.w

        transformed_pose = transform_pose(my_pose, "raspicam", "map")'''

        marker.pose.position.x = transform.transform.translation.x
        marker.pose.position.y = transform.transform.translation.y - 0.2
        marker.pose.position.z = transform.transform.translation.z
        marker.pose.orientation.x = transform.transform.rotation.x
        marker.pose.orientation.y = transform.transform.rotation.y
        marker.pose.orientation.z = transform.transform.rotation.z
        marker.pose.orientation.w = transform.transform.rotation.w

        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()
        
        # 1Hz
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            connections = m_pub.get_num_connections()
            if connections > 0:
                m_pub.publish(marker)
                print("Published marker")
                published.append(marker.id)
                break
            rate.sleep()
        
    

def main():
    rospy.init_node('publish_markers', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
