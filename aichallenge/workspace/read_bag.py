import os
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print("Please run this script inside the autoware container with ROS 2 sourced!")
    sys.exit(1)

def main():
    bag_path = '/aichallenge/workspace/skidpad_bag'
    
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    accel_type_string = type_map.get('/localization/acceleration')
    if not accel_type_string:
        print("No acceleration topic found in the bag!")
        return
        
    accel_type = get_message(accel_type_string)
    
    max_ay = 0.0
    min_ax = 0.0 # for max braking
    
    imu_type_string = type_map.get('/sensing/imu/imu_data')
    if imu_type_string:
        imu_type = get_message(imu_type_string)
    else:
        imu_type = None
        print("No IMU topic found!")

    vel_type_string = type_map.get('/vehicle/status/velocity_status')
    if vel_type_string:
        vel_type = get_message(vel_type_string)
    else:
        vel_type = None

    count = 0
    max_yaw_rate = 0.0
    
    last_vel = 0.0
    max_ay_kinematic = 0.0
    
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == '/localization/acceleration':
            msg = deserialize_message(data, accel_type)
            ax = msg.accel.accel.linear.x
            
            if ax < min_ax:
                min_ax = ax
                
            count += 1
            
        elif topic == '/vehicle/status/velocity_status' and vel_type:
            msg = deserialize_message(data, vel_type)
            last_vel = msg.longitudinal_velocity
            
        elif topic == '/sensing/imu/imu_data' and imu_type:
            msg = deserialize_message(data, imu_type)
            ay_raw = msg.linear_acceleration.y
            omega_z = msg.angular_velocity.z
            
            if abs(ay_raw) > max_ay:
                max_ay = abs(ay_raw)
                
            ay_kin = abs(last_vel * omega_z)
            if ay_kin > max_ay_kinematic:
                max_ay_kinematic = ay_kin

    print(f"Total messages processed: {count}")
    print(f"Max Raw IMU Lateral Accel (Includes impacts): {max_ay:.3f} m/s^2")
    print(f"Max Kinematic Lateral Accel (v * omega, filters impacts): {max_ay_kinematic:.3f} m/s^2  --> Lateral mue = {max_ay_kinematic/9.81:.3f}")
    print(f"Max Braking Deceleration (/localization/acceleration): {min_ax:.3f} m/s^2  --> Longitudinal mue = {abs(min_ax)/9.81:.3f}")

if __name__ == "__main__":
    main()
