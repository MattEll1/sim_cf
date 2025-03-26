#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from crazyflie_driver.msg import GenericLogData
import math
import csv
import os
from datetime import datetime

# Global variables to store current attitude
current_roll = 0.0
current_pitch = 0.0
current_yaw = 0.0
log_data = []  # To store attitude data for logging

def attitude_callback(data):
    """Callback function for the local_position topic to get attitude"""
    global current_roll, current_pitch, current_yaw, log_data
    
    # The values are already in degrees, no need to convert
    roll_deg = data.values[3]
    pitch_deg = data.values[4]
    yaw_deg = data.values[5]
    
    # Convert to radians for consistency with ROS standards
    current_roll = math.radians(roll_deg)
    current_pitch = math.radians(pitch_deg)
    current_yaw = math.radians(yaw_deg)
    
    # Store attitude data with timestamp
    timestamp = rospy.Time.now().to_sec()
    log_data.append([timestamp, current_roll, current_pitch, current_yaw, 
                     roll_deg, pitch_deg, yaw_deg])
    
    # Only print occasionally to avoid flooding the console
    if rospy.get_time() % 0.5 < 0.1:  # Print roughly twice per second
        rospy.loginfo("Attitude - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
                      roll_deg, pitch_deg, yaw_deg)

def save_log_data():
    """Save logged attitude data to a CSV file"""
    if not log_data:
        rospy.logwarn("No attitude data to save")
        return
    
    # Create logs directory if it doesn't exist
    log_dir = os.path.expanduser("/code/attitude_logs")
    if not os.path.exists(log_dir):
        os.makedirs(log_dir)
    
    # Generate filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(log_dir, f"attitude_log_{timestamp}.csv")
    
    # Write data to CSV
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Timestamp", "Roll(rad)", "Pitch(rad)", "Yaw(rad)", 
                         "Roll(deg)", "Pitch(deg)", "Yaw(deg)"])
        writer.writerows(log_data)
    
    rospy.loginfo("Attitude log saved to %s", filename)

def main():
    # Initialize the node and publisher
    rospy.init_node('drone_controller')
    pub = rospy.Publisher('/cf1/direct_control', Float32MultiArray, queue_size=10)
    
    # Subscribe to get actual drone attitude
    attitude_sub = rospy.Subscriber('/cf1/local_position', GenericLogData, attitude_callback)
    
    # Give time for the publisher to connect
    rospy.sleep(1.0)
    
    # Create a message
    msg = Float32MultiArray()
    
    # Set publishing rate
    rate = rospy.Rate(50)  # 50Hz to match controller rate
    
    # Track time
    start_time = rospy.Time.now()
    active_duration = rospy.Duration(2.0)  # 5 seconds of active control
    
    rospy.loginfo("Starting to send direct control commands for 5 seconds...")
    
    try:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = current_time - start_time
            
            # Phase 1: Thrust only (first 2 seconds)
            if elapsed < rospy.Duration(2.0):
                msg.data = [0.0, 0.0, 0.0, 47000]  # [roll, pitch, yaw, throttle]
                if elapsed.to_sec() % 1 < 0.1:
                    rospy.loginfo("Phase 1 - Thrust only: %.1f seconds remaining", 
                                 2.0 - elapsed.to_sec())
            
            # Phase 2: Pitch with thrust (2-4 seconds)
            elif elapsed < rospy.Duration(4.0):
                msg.data = [5.0, 0.0, 0.0, 50000]  # [roll, pitch, yaw, throttle]
                if (elapsed.to_sec() - 2.0) % 1 < 0.1:
                    rospy.loginfo("Phase 2 - Roll: %.1f seconds remaining", 
                                 4.0 - elapsed.to_sec())
            
            # Phase 3: Turn off everything (after 4 seconds)
            else:
                msg.data = [0.0, 0.0, 0.0, 0.0]  # All zeros
                if elapsed.to_sec() < 4.1:  # Log only once when transitioning
                    rospy.loginfo("Phase 3 - All controls off")
            
            # Publish the message
            pub.publish(msg)
            
            # Stop after 10 seconds total
            if elapsed.to_sec() > 4.0:
                break
                
            # Maintain the publishing rate
            rate.sleep()
            
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt detected")
    finally:
        # Make sure we send zeros when terminating
        msg.data = [0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)
        rospy.loginfo("Stopping drone controller")
        
        # Save attitude log data
        save_log_data()

if __name__ == '__main__':
    main()