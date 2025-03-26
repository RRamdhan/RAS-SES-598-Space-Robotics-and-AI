#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = None

def state_cb(msg):
    global current_state
    current_state = msg

if __name__ == "__main__":
    rospy.init_node("offboard_fly_up_node")

    # Subscribers and Publishers
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)

    # Service proxies
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    while not rospy.is_shutdown() and current_state is None:
        rospy.loginfo("Waiting for current state...")
        rate.sleep()

    # Define initial pose
    pose = PoseStamped()
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2  # Takeoff to 2m initially

    # Send a few setpoints before switching modes
    for _ in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # Set OFFBOARD mode and arm
    set_mode_client(custom_mode="OFFBOARD")
    arming_client(True)
    rospy.loginfo("Drone armed and in OFFBOARD mode")

    rospy.sleep(5)

    # Now command drone to ascend 100 meters
    pose.pose.position.z = 100
    rospy.loginfo("Flying up to 100 meters")

    for _ in range(400):  # ~20 seconds of publishing
        local_pos_pub.publish(pose)
        rate.sleep()
