#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
import random
from visualization_msgs.msg import Marker

class Navigation(Node):
    def __init__(self):
        super().__init__('navigation')

        self.latest_image = None
        self.latest_depth_image = None
        self.latest_map = None
        self.fx = None
        self.fy = None
        self.cx0 = None
        self.cy0 = None
        self.bridge = CvBridge()
        self.camera_subscriber = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.depth_camera_subscriber = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_image_callback,
            10
        )

        self.depth_camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_camera_info_callback,
            10
        )

        self.camera_info_subscriber = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.map_callback,
            10
        )

        self.image_publisher = self.create_publisher(Image, '/navigation/debug_image', 10)
        self.marker_publisher = self.create_publisher(Marker, '/navigation/goal_marker', 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_goal_type = None # "yellow" or "random"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(1.0, self.timer_callback)
        self.kernel = np.ones((5, 5), np.uint8)
        self.color_dict = {
            "red": {"hsv_low": (0, 50, 40), "hsv_high": (10, 255, 255)},
            "green": {"hsv_low": (35, 50, 40), "hsv_high": (85, 255, 255)},
            "blue": {"hsv_low": (100, 50, 40), "hsv_high": (140, 255, 255)},
            "yellow": {"hsv_low": (20, 50, 40), "hsv_high": (35, 255, 255)},
        }
    
    def image_callback(self, msg):
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_image_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def depth_camera_info_callback(self, msg):
        pass

    def camera_info_callback(self, msg):
        k = msg.k
        self.fx = k[0]
        self.fy = k[4]
        self.cx0 = k[2]
        self.cy0 = k[5]
    
    def map_callback(self, msg):
        self.latest_map = msg

    def timer_callback(self):
        if self.latest_image is None:
            self.get_logger().info("No latest image received yet")
            return

        if self.latest_depth_image is None:
            self.get_logger().info("No latest depth image received yet")
            return

        if self.fx is None or self.fy is None or self.cx0 is None or self.cy0 is None:
            self.get_logger().info("Waiting for camera intrinsic parameters")
            return
        
        if self.latest_map is None:
            self.get_logger().info("Waiting for map data")
            return

        ## Checking if Navigation2 is available
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Waiting for /navigate_to_pose action server")
            return
    
        self.get_logger().info("Image received, processing for part detection")
        parts = self.detect_parts(self.latest_image)
        self.send_navigation_goal(parts)

    def detect_parts(self, image):
        debug_image = image.copy()
        parts: list[dict] = []
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        for _, hsv_range in self.color_dict.items():
            mask = cv2.inRange(hsv_image, hsv_range["hsv_low"], hsv_range["hsv_high"])
            colour_name = list(self.color_dict.keys())[list(self.color_dict.values()).index(hsv_range)]

            # Apply morphological operations to clean up the mask
            #mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)
            #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

            # Find contours in each mask and detect the shape (sphere, cube, cylinder)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # For each contour above a minimum area:
            #         - compute centroid using cv2.moments
            #         - compute orientation using cv2.minAreaRect
            for contour in contours:
                shape = self.get_shape(contour)
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                else:
                    cx, cy = 0, 0

                rect = cv2.minAreaRect(contour)
                angle = rect[2]
                if angle < -45:
                    angle += 90

                cv2.drawContours(debug_image, [contour], -1, (0, 255, 0), 2)
                cv2.circle(debug_image, (int(cx), int(cy)), 5, (255, 0, 0), -1)
                cv2.putText(
                    debug_image, 
                    f"{colour_name}, area={float(cv2.contourArea(contour)):.1f}, shape={shape}", 
                    (int(cx) + 8, int(cy) -8), 
                    cv2.FONT_HERSHEY_SIMPLEX, 
                    0.5, 
                    (255, 255, 255), 
                    1,
                    cv2.LINE_AA
                )
                # Return the list of detection dicts
                parts.append(
                        {"color": str(colour_name), "cx": float(cx), "cy": float(cy), "angle_deg": float(angle), "area": float(cv2.contourArea(contour)), "shape": shape}
                    )

        debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "camera_link"
        self.image_publisher.publish(debug_msg)
        return parts

    def get_shape(self, contour):
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)
        vertices = len(approx)

        if perimeter == 0:
            return "unknown"
        
        if vertices == 3:
            return "triangle"
        if vertices == 4:
            return "cube"

        circularity = 4 * np.pi * (area / (perimeter * perimeter))

        if circularity > 0.8:
            return "sphere"
        elif circularity > 0.5:
            return "cylinder"

    def navigate_to_random_goal(self):
        width = self.latest_map.info.width
        height = self.latest_map.info.height
        free_cells = []
        d = 3

        for x in range(width):
            for y in range(height):
                ## ramdomly sample points until we find a free cell (value 0) to navigate to
                index = y * width + x
                if self.latest_map.data[index] == 0:
                    free_cells.append((x, y))

        if not free_cells:
            self.get_logger().warn("No free cells found in the map.")
            return

        x, y = random.choice(free_cells)

        resolution = self.latest_map.info.resolution
        origin = self.latest_map.info.origin.position

        world_x = origin.x + (x + 0.5) * resolution
        world_y = origin.y + (y + 0.5) * resolution

        self.get_logger().info(f"Navigating to random free cell at map coordinates: ({world_x:.2f}, {world_y:.2f})")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = world_x
        goal_msg.pose.pose.position.y = world_y
        goal_msg.pose.pose.orientation.w = 1.0

        self.publish_goal_marker(world_x, world_y)

        self.current_goal_type = "random"
        self.is_navigating = True
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def navigate_to_yellow_goal(self, part):
        x_cam = part["cx"] 
        y_cam = part["cy"]
        x_norm = (x_cam - self.cx0) / self.fx
        y_norm = (y_cam - self.cy0) / self.fy

        depth_value = float(self.latest_depth_image[int(y_cam), int(x_cam)])
        if not np.isfinite(depth_value) or depth_value <= 0.0:
            self.get_logger().warn(f"Invalid depth value at pixel ({x_cam}, {y_cam}): {depth_value}")
            return

        X = depth_value * x_norm
        Y = depth_value * y_norm
        Z = depth_value

        ### Convert to tf coordinates if necessary (e.g., from camera frame to map frame) using tf transformations
        # Wrap the point in a PointStamped message for transformation
        point_stamped = PointStamped()
        point_stamped.header.stamp = rclpy.time.Time().to_msg()
        point_stamped.header.frame_id = "depth_camera_link"
        point_stamped.point.x = Z
        point_stamped.point.y = -X
        point_stamped.point.z = -Y

        # Transform the point to the "map" frame using tf2
        try:
            transformed_point = self.tf_buffer.transform(point_stamped, "map", timeout=rclpy.duration.Duration(seconds=0.5))
            x_world = transformed_point.point.x
            y_world = transformed_point.point.y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Failed to transform from {point_stamped.header.frame_id} to map: {e}")
                return

        ## Send it NavigateToPose action goal
        self.get_logger().info("Sending Navigation pose")
        self.get_logger().info(f"Detected yellow part at world coordinates: ({x_world:.2f}, {y_world:.2f}) with angle {part['angle_deg']:.2f} degrees")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x_world #- 0.5 # Move 0.5 meters back from the part to avoid collision
        goal_msg.pose.pose.position.y = y_world #- 0.5 # Move 0.5 meters back from the part to avoid collision
        goal_msg.pose.pose.orientation.w = 1.0

        self.publish_goal_marker(x_world, y_world)

        self.is_navigating = True
        self.current_goal_type = "yellow"
        future = self.nav_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)


    def send_navigation_goal(self, parts):
        yellow_parts = [p for p in parts if p["color"] == "yellow" and p["shape"] == "sphere"]

        if yellow_parts:
            best_part = max(yellow_parts, key=lambda part: part["area"])
            self.get_logger().info(f"Best part selected: color={best_part['color']}, cx={best_part['cx']:.2f}, cy={best_part['cy']:.2f}, angle={best_part['angle_deg']:.2f} degrees, area={best_part['area']:.2f}")

            if self.is_navigating and self.current_goal_type == "random":
                self.current_goal_handle.cancel_goal_async()
                self.is_navigating = False

            if not self.is_navigating:
                self.navigate_to_yellow_goal(best_part)
            
            return
        
        if not self.is_navigating:
            self.navigate_to_random_goal()            

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "goals"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        self.marker_publisher.publish(marker)

    def goal_response_callback(self, future):
        goal_handle = future.result() 

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected")
            self.is_navigating = False
            self.current_goal_handle = None
            self.current_goal_type = None
            return   
        
        self.current_goal_handle = goal_handle
        goal_type = self.current_goal_type
        self.get_logger().info("Goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self.goal_result_callback(future, goal_type)
        ) 

    def goal_result_callback(self, future, goal_type):
        result = future.result().result
        status = future.result().status

        ## To ensure the callback belongs to the current active goal
        if goal_type != self.current_goal_type:
            self.get_logger().info("Received result for an old goal, ignoring.")
            return

        self.get_logger().info(f"Goal finished with status: {status}")
        self.is_navigating = False
        self.current_goal_handle = None
        self.current_goal_type = None


def main(args=None):
    rclpy.init(args=args)
    node = DetectShape()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
