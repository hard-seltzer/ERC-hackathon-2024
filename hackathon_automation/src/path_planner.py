import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose
from obstacle_detection import Maze
import heapq
import math

class Node:
    def __init__(self, position, g_cost, h_cost, parent=None):
        self.position = position
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.f_cost = g_cost + h_cost
        self.parent = parent

    def __lt__(self, other):
        return self.f_cost < other.f_cost

def heuristic(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

def get_neighbors(maze, node):
    neighbors = []
    for dx, dy in [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
        new_pos = (node.position[0] + dx, node.position[1] + dy)
        if not maze.isPointInObstacles(new_pos[0], new_pos[1]):
            neighbors.append(new_pos)
    return neighbors

def a_star(maze, start, goal):
    start_node = Node(start, 0, heuristic(start, goal))
    open_list = [start_node]
    closed_set = set()

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.position == goal:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        closed_set.add(current_node.position)

        for neighbor_pos in get_neighbors(maze, current_node):
            if neighbor_pos in closed_set:
                continue

            neighbor = Node(neighbor_pos, current_node.g_cost + 1, heuristic(neighbor_pos, goal), current_node)

            if neighbor not in open_list:
                heapq.heappush(open_list, neighbor)
            else:
                idx = open_list.index(neighbor)
                if open_list[idx].g_cost > neighbor.g_cost:
                    open_list[idx] = neighbor
                    heapq.heapify(open_list)

    return None

def plan_path(maze, waypoints):
    full_path = []
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        goal = waypoints[i + 1]
        path_segment = a_star(maze, start, goal)
        if path_segment is None:
            print(f"No path found between {start} and {goal}")
            return None
        full_path.extend(path_segment[:-1])  # Exclude the last point to avoid duplication
    full_path.append(waypoints[-1])  # Add the final goal
    return full_path

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('path_planner')
    path_publisher = node.create_publisher(PoseArray, '/planned_path', 10)

    maze = Maze()
    waypoints = [
        (109, 296), (155, 205), (299, 142), (506, 536), (239, 643), (193, 329)
    ]

    path = plan_path(maze, waypoints)

    if path:
        pose_array = PoseArray()
        pose_array.header.frame_id = 'map'
        for point in path:
            pose = Pose()
            gazebo_point = maze.point_transform(point)
            pose.position.x = gazebo_point[0]
            pose.position.y = gazebo_point[1]
            pose_array.poses.append(pose)

        path_publisher.publish(pose_array)
        node.get_logger().info('Published planned path')
    else:
        node.get_logger().error('Failed to find a complete path')

    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()