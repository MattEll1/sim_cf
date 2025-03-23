#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time

def main():
    # Initialize the node and publisher
    rospy.init_node('drone_controller')
    pub = rospy.Publisher('/cf1/direct_control', Float32MultiArray, queue_size=10)
    
    # Give time for the publisher to connect
    rospy.sleep(1.0)
    
    # Create a message
    msg = Float32MultiArray()
    
    # Set publishing rate
    rate = rospy.Rate(50)  # 50Hz to match controller rate
    
    # Track time
    start_time = rospy.Time.now()
    active_duration = rospy.Duration(5.0)  # 5 seconds of active control
    
    print("Starting to send direct control commands for 5 seconds...")
    
    try:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = current_time - start_time
            
            if elapsed < active_duration:
                # Active control phase (first 5 seconds)
                msg.data = [1.0, 0.0, 0.0, 0.4]  # [roll, pitch, yaw, throttle]
                print("Active control: %d seconds remaining" % (5 - elapsed.to_sec()))
            else:
                # Zero control phase (after 5 seconds)
                msg.data = [0.0, 0.0, 0.0, 0.0]  # All zeros
                if elapsed.to_sec() < active_duration.to_sec() + 1.0:
                    print("Control set to zero")
            
            # Publish the message
            pub.publish(msg)
            
            # Maintain the publishing rate
            rate.sleep()
            
    except KeyboardInterrupt:
        # Make sure we send zeros when terminating
        msg.data = [0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)
        print("Stopping drone controller")

if __name__ == '__main__':
    main()