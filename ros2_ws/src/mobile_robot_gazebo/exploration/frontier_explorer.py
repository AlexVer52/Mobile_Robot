#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
import random

class Exploration(Node):
    def __init__(self):
        super().__init__('exploration')
        self.latest_map = None
        self.is_navigating = False
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.timer = self.create_timer(1.0, self.timer_callback)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def map_callback(self, msg):
            self.latest_map = msg
    
    def timer_callback(self):
        if self.latest_map is None:
            return
        if self.is_navigating == True:
            return

        ## Checking if Navigation2 is available
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Waiting for /navigate_to_pose action server")
            return
        
        frontiers = self.detect_frontiers()

        if not frontiers:
            self.get_logger().info("No frontiers found")
            return
        goal = random.choice(frontiers)
        self.send_goal(goal)

    def detect_frontiers(self):
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        goals = []

        for x in range(width):
            for y in range(height):
                index = y * width + x
                if self.latest_map.data[index] == 0:
                    # Check neighbors
                    neighbors = [
                        (x-1, y), (x+1, y), 
                        (x, y-1), (x, y+1)
                    ]
                    for nx, ny in neighbors:
                        if 0 <= nx < width and 0 <= ny < height:
                            n_index = ny * width + nx
                            if self.latest_map.data[n_index] == -1:
                                goals.append((x, y))
        return goals
        


    def send_goal(self, goal):
        x_cell, y_cell = goal

        resolution = self.latest_map.info.resolution
        origin = self.latest_map.info.origin.position

        world_x = origin.x + (x_cell + 0.5) * resolution
        world_y = origin.y + (y_cell + 0.5) * resolution

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.is_navigating = True
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result() 

        if not goal_handle.accepted:
            self.get_logger().warn("Frontier goal rejected")
            self.is_navigating = False
            return   
        
        self.get_logger().info("Frontier goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback) 

    def goal_result_callback(self, future):
        result = future.result().result
        status = future.result().status 

        self.get_logger().info(f"Frontier goal finished with status: {status}")
        self.is_navigating = False


def main(args=None):
    rclpy.init(args=args)
    node = Exploration()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
      main()