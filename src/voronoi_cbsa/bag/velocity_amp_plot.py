import rosbag
import numpy as np
import matplotlib.pyplot as plt

def calculate_2d_velocity(bag_path, topic):
    # Open the bag file
    bag = rosbag.Bag(bag_path)
    
    # Lists to store data
    timestamps = []
    velocities_2d = []
    
    # Extract velocity data from the bag
    for topic, msg, t in bag.read_messages(topics=[topic]):
        # Get linear velocities in x and y directions
        vx = msg.GT_twist.linear.x
        vy = msg.GT_twist.linear.y
        
        # Calculate 2D velocity magnitude using Pythagorean theorem
        velocity_2d = np.sqrt(vx**2 + vy**2)
        
        # Store timestamp and velocity
        timestamps.append(msg.header.stamp.to_sec())
        velocities_2d.append(velocity_2d)
    
    bag.close()
    
    # Plot the results
    plt.figure(figsize=(10, 6))
    plt.plot(timestamps, velocities_2d, label='2D Velocity')
    plt.xlabel('Time (seconds)')
    plt.ylabel('Velocity (m/s)')
    plt.title('2D Velocity Over Time')
    plt.grid(True)
    plt.legend()
    plt.show()
    
    # Calculate and print average velocity
    avg_velocity = np.mean(velocities_2d)
    print(f"Average 2D velocity: {avg_velocity:.3f} m/s")
    
    return timestamps, velocities_2d

# Example usage
bag_path = '/home/weiggen/wei_research_ws/src/voronoi_cbsa/bag/d3.bag'
topic = '/iris_1/THEIF/Plot'

timestamps, velocities = calculate_2d_velocity(bag_path, topic)