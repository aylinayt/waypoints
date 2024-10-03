import csv
import argparse
import carla
import time

def visualize_waypoints(waypoints):
    """
    Visualizes waypoints in the CARLA simulation environment.
    Args:
    waypoints (list of tuples): List of waypoints where each waypoint is a tuple (x, y,
    z).
    """
    # Connect to the CARLA client
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    # Iterate over the waypoints and visualize each one in the CARLA world
    for waypoint in waypoints:
        # Create a CARLA Location object with x, y, z coordinates
        # Note: The y-coordinate is negated to match CARLA's coordinate system
        location = carla.Location(x=waypoint[0], y=-waypoint[1], z=waypoint[2])
        # Draw a red 'O' at the waypoint location in the CARLA world
        world.debug.draw_string(location, 'O', draw_shadow=False,
        color=carla.Color(r=255, g=0, b=0), life_time=10.0,
        persistent_lines=True)
        # Print the waypoint coordinates being visualized
        print(f"Visualizing waypoint at x={waypoint[0]}, y={-waypoint[1]}, z={waypoint[2]}")
        # Note the flipped y-coordinate

if __name__ == '__main__':
    waypoints = []
    # Read waypoints from a CSV file
    with open('waypoints.csv', 'r') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader) # Skip the CSV header
    # Load each row in the CSV file and append to the waypoints list
        for row in csvreader:
            timestamp, x, y, z, yaw, speed = map(float, row)
            waypoints.append((x, y, z))
            print(f"Loaded waypoint: x={x}, y={y}, z={z}, yaw={yaw}, speed={speed}")
        # Visualize the loaded waypoints in the CARLA simulation
    visualize_waypoints(waypoints)