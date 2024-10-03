import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from transforms3d.euler import quat2euler
import csv
import argparse
import carla
import time

class WaypointCollectorNode(Node):
    def __init__(self, filename):
        super().__init__('waypoint_collector_node')
        self.filename = filename
        self.current_speed = 0.0
        self.data_rows = []

        # Subscribe to the speedometer topic to get the vehicle's speed
        self.speed_subscriber = self.create_subscription(Float32,
        'carla/ego_vehicle/speedometer', self.speed_callback, 10)
        # Subscribe to the odometry topic to get the vehicle's position and orientation
        self.odometry_subscriber = self.create_subscription(Odometry,
        'carla/ego_vehicle/odometry', self.odometry_callback, 10)
        self.traffic_light_timer = self.create_timer(1.0, self.traffic_light_timer_callback)
        self.carla_client = carla.Client('localhost', 2000)
        self.carla_client.set_timeout(10.0)
        self.carla_world = self.carla_client.get_world()
        self.set_traffic_lights_to_green()
        self.enable_autodrive()


    def set_traffic_lights_to_green(self):
        """Set all traffic lights in the simulation to green."""
        traffic_lights = self.carla_world.get_actors().filter('traffic.traffic_light')
        for traffic_light in traffic_lights:
            traffic_light.set_state(carla.TrafficLightState.Green)
            traffic_light.set_green_time(99999) # Set a very long green time

    def enable_autodrive(self):
        """Enable autodrive for the first vehicle found in the simulation."""
        max_retries = 10
        retries = 0
        while retries < max_retries:
            vehicles = self.carla_world.get_actors().filter('vehicle.*')
            if vehicles:
                ego_vehicle = vehicles[0]
                ego_vehicle.set_autopilot(True)
                self.get_logger().info("Autodrive enabled for the ego vehicle.")
                return
            else:
                self.get_logger().info("Waiting for ego vehicle to spawn...")
                retries += 1
                time.sleep(1)
                self.get_logger().error("No vehicles found in the simulation after waiting.")

    def speed_callback(self, speed_data):
        """Callback function to update the current speed."""
        self.current_speed = speed_data.data


    def odometry_callback(self, odometry_data):
        """Callback function to collect and store odometry data."""
        x_position = odometry_data.pose.pose.position.x
        y_position = odometry_data.pose.pose.position.y
        z_position = odometry_data.pose.pose.position.z
        roll, pitch, yaw = quat2euler([odometry_data.pose.pose.orientation.w,
        odometry_data.pose.pose.orientation.x,
        odometry_data.pose.pose.orientation.y,
        odometry_data.pose.pose.orientation.z])
        time_stamp = odometry_data.header.stamp.sec + odometry_data.header.stamp.nanosec *1e-9
        self.data_rows.append([time_stamp, x_position, y_position, z_position, yaw,
        self.current_speed])

    def traffic_light_timer_callback(self):
        """Timer callback to periodically reset all traffic lights to green."""
        self.set_traffic_lights_to_green()

    def __del__(self):
        """Destructor to write collected data to a CSV file upon node destruction."""
        self.get_logger().info("Writing collected data to CSV file.")
        field_names = ['time', 'x', 'y', 'z', 'yaw', 'speed']
        with open(self.filename + ".csv", 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(field_names) # Write CSV header
            csv_writer.writerows(self.data_rows) # Write data rows

        self.get_logger().info("Finished writing data to CSV file.")

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description="Waypoint data collection.")
    parser.add_argument('-f', '--filename', default="waypoints", help="waypoint.csv")
    args = parser.parse_args()
    if args.filename is None:
        raise ValueError("Filename not specified. Please use --help or -h for more information.")
    waypoint_collector_node = WaypointCollectorNode(args.filename)
    try:
        rclpy.spin(waypoint_collector_node)
    except KeyboardInterrupt:
        pass
        
    waypoint_collector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()