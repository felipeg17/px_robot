import rospy
import math
import tf2_ros
import tf.transformations as tf
import numpy as np
import geometry_msgs.msg

if __name__ == '__main__':
    np.set_printoptions(suppress=True)
    rospy.init_node('tf2_px_listener')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('world', 'tcp_link', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        #print(trans.transform.rotation)
        quat = [trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
        eu_ang = tf.euler_from_quaternion(quat)
        pos = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        scale = np.ones(3)
        shear = np.zeros(3)
        angles = eu_ang
        trans = pos
        persp = [0,0,0,1]
        T1 = tf.compose_matrix(scale, shear, angles, trans, persp)
        print(T1)
        rate.sleep()