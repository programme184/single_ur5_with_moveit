#!/usr/bin/env python  
import rospy
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # Listen to the transform from "base_link" to "dummy"
            (trans_dummy, rot_dummy) = listener.lookupTransform("base_link", "dummy", rospy.Time(0))
            print("Transform from 'base_link' to 'dummy':")
            print("Translation:", trans_dummy)
            print("Rotation:", rot_dummy)

            # Listen to the transform from "base_link" to "test"
            (trans_test, rot_test) = listener.lookupTransform("base_link", "test", rospy.Time(0))
            print("\nTransform from 'base_link' to 'test':")
            print("Translation:", trans_test)
            print("Rotation:", rot_test)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()


