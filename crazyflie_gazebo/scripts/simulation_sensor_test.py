#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
import time
import csv
import os
from datetime import datetime
from gazebo_msgs.msg import ModelStates
import numpy as np

# Global variable to store IMU data
imu_data = []
# Track if we're receiving IMU data
receiving_imu = False

def imu_callback(msg):
    """Callback function for IMU data"""
    global imu_data, receiving_imu
    receiving_imu = True
    
    # Get timestamp
    timestamp = rospy.Time.now().to_sec()
    
    # Get acceleration data - this will depend on the exact topic format
    try:
        # Find the crazyflie model in the ModelStates message
        if 'cf1' in msg.name:
            index = msg.name.index('cf1')
            # Get linear acceleration data
            accel_x = msg.twist[index].linear.x
            accel_y = msg.twist[index].linear.y
            accel_z = msg.twist[index].linear.z
            
            # Get angular velocity data
            gyro_x = msg.twist[index].angular.x
            gyro_y = msg.twist[index].angular.y
            gyro_z = msg.twist[index].angular.z
            
            # Store the data
            imu_data.append([timestamp, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z])
    except (ValueError, AttributeError, IndexError) as e:
        rospy.logwarn(f"Error processing IMU data: {e}")

def save_imu_data(test_description="flight_test"):
    """Save IMU data to CSV file"""
    global imu_data
    if not imu_data:
        rospy.logwarn("No IMU data to save")
        return
    
    # Create directory if it doesn't exist
    output_dir = os.path.join(os.path.dirname(os.path.realpath(__file__)), "../test_output")
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    
    # Create filename with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = os.path.join(output_dir, f"sim_imu_data_{test_description}_{timestamp}.csv")
    
    # Write data to CSV
    with open(filename, 'w') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])
        writer.writerows(imu_data)
    
    rospy.loginfo(f"Saved IMU data to {filename}")
    return filename

def main():
    global imu_data
    
    # Initialize the node and publishers/subscribers
    rospy.init_node('drone_controller_with_imu_logging')
    pub = rospy.Publisher('/cf1/direct_control', Float32MultiArray, queue_size=10)
    
    # Subscribe to IMU data
    # Note: Try different topics if this one doesn't work
    rospy.Subscriber('/gazebo/model_states', ModelStates, imu_callback)
    
    # Give time for connections to establish
    rospy.sleep(1.0)
    
    # Create message for control commands
    msg = Float32MultiArray()
    
    # Set publishing rate
    rate = rospy.Rate(50)  # 50Hz to match controller rate
    
    # Track time
    start_time = rospy.Time.now()
    idle_duration = rospy.Duration(10.0)       # 10 seconds idle
    active_duration = rospy.Duration(10.0)     # 10 seconds active
    
    print("Starting test sequence...")
    print("Phase 1: Idle for 10 seconds")
    
    try:
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            elapsed = current_time - start_time
            
            if elapsed < idle_duration:
                # Idle phase (first 10 seconds)
                msg.data = [0.0, 0.0, 0.0, 0.0]  # [roll, pitch, yaw, throttle]
                
            elif elapsed < (idle_duration + active_duration):
                # Active control phase (next 10 seconds)
                if elapsed.to_sec() - idle_duration.to_sec() < 0.1:  # Just entered active phase
                    print("Phase 2: 50% throttle for 10 seconds")
                
                msg.data = [0.0, 0.0, 0.0, 30000]  # 50% throttle (approx.)
                
            else:
                # Test complete
                msg.data = [0.0, 0.0, 0.0, 0.0]  # All zeros
                
                # If we've just finished, save the data
                if elapsed.to_sec() < (idle_duration + active_duration).to_sec() + 0.1:
                    print("Test complete. Saving data...")
                    save_imu_data("idle_then_50pct_throttle")
                    
                # Exit after a brief delay to ensure zero commands are sent
                if elapsed.to_sec() > (idle_duration + active_duration).to_sec() + 1.0:
                    break
            
            # Publish the control message
            pub.publish(msg)
            
            # Display IMU data status
            if not receiving_imu and elapsed.to_sec() > 2.0:
                print("WARNING: Not receiving IMU data!")
            
            # Maintain the publishing rate
            rate.sleep()
            
    except KeyboardInterrupt:
        print("Test interrupted.")
    finally:
        # Make sure we send zeros when terminating
        msg.data = [0.0, 0.0, 0.0, 0.0]
        pub.publish(msg)
        
        # Save any collected data
        if imu_data:
            save_imu_data("interrupted_test")
            
        print("Test ended.")

if __name__ == '__main__':
    main()