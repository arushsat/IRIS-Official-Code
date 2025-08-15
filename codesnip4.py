#!/usr/bin/env python3
"""
IRIS Robot - Obstacle Detection and Navigation System
Uses LIDAR sensor data for obstacle detection and autonomous navigation.
"""

import numpy as np
import math
import time
import json
import logging
from datetime import datetime
import argparse
from typing import List, Tuple, Dict, Optional
import matplotlib.pyplot as plt
from dataclasses import dataclass
import threading
import queue

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('iris_navigation.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)

@dataclass
class Point:
    """2D point representation"""
    x: float
    y: float
    
    def distance_to(self, other: 'Point') -> float:
        """Calculate distance to another point"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Point') -> float:
        """Calculate angle to another point in radians"""
        return math.atan2(other.y - self.y, other.x - self.x)

@dataclass
class Obstacle:
    """Obstacle representation"""
    position: Point
    radius: float
    confidence: float
    obstacle_type: str  # 'static', 'dynamic', 'unknown'
    timestamp: float

@dataclass
class NavigationCommand:
    """Navigation command for the robot"""
    linear_velocity: float  # m/s
    angular_velocity: float  # rad/s
    direction: str  # 'forward', 'backward', 'left', 'right', 'stop'
    confidence: float
    timestamp: float

class LIDARSimulator:
    """Simulates LIDAR sensor for testing purposes"""
    
    def __init__(self, max_range: float = 12.0, angular_resolution: float = 1.0):
        self.max_range = max_range
        self.angular_resolution = angular_resolution  # degrees
        self.num_angles = int(360 / angular_resolution)
        self.angles = np.linspace(0, 2*np.pi, self.num_angles, endpoint=False)
        
        # Simulated obstacles for testing
        self.simulated_obstacles = [
            {'x': 3.0, 'y': 1.0, 'radius': 0.5},
            {'x': -2.0, 'y': 2.0, 'radius': 0.8},
            {'x': 1.5, 'y': -1.5, 'radius': 0.3},
        ]
    
    def get_scan_data(self) -> np.ndarray:
        """Get simulated LIDAR scan data"""
        distances = np.full(self.num_angles, self.max_range)
        
        # Add simulated obstacles
        for obstacle in self.simulated_obstacles:
            for i, angle in enumerate(self.angles):
                # Calculate distance to obstacle at this angle
                dx = obstacle['x'] - 0  # Robot at origin
                dy = obstacle['y'] - 0
                
                # Calculate angle to obstacle
                obstacle_angle = math.atan2(dy, dx)
                if obstacle_angle < 0:
                    obstacle_angle += 2*np.pi
                
                # Find closest angle in our scan
                angle_diff = abs(angle - obstacle_angle)
                if angle_diff > np.pi:
                    angle_diff = 2*np.pi - angle_diff
                
                if angle_diff < np.radians(10):  # Obstacle visible within 10 degrees
                    # Calculate distance to obstacle surface
                    distance_to_center = math.sqrt(dx**2 + dy**2)
                    distance_to_surface = distance_to_center - obstacle['radius']
                    
                    if distance_to_surface > 0 and distance_to_surface < distances[i]:
                        distances[i] = distance_to_surface
        
        # Add some noise
        noise = np.random.normal(0, 0.1, self.num_angles)
        distances += noise
        distances = np.clip(distances, 0.1, self.max_range)
        
        return distances
    
    def get_obstacle_positions(self) -> List[Dict]:
        """Get current obstacle positions for visualization"""
        return self.simulated_obstacles

class ObstacleDetector:
    """Detects and tracks obstacles from LIDAR data"""
    
    def __init__(self, min_distance: float = 0.5, cluster_threshold: float = 0.3):
        self.min_distance = min_distance
        self.cluster_threshold = cluster_threshold
        self.detected_obstacles: List[Obstacle] = []
        self.obstacle_history: List[List[Obstacle]] = []
        
    def detect_obstacles(self, distances: np.ndarray, angles: np.ndarray) -> List[Obstacle]:
        """Detect obstacles from LIDAR scan data"""
        obstacles = []
        
        # Find points below minimum distance threshold
        close_points = []
        for i, distance in enumerate(distances):
            if distance < self.min_distance:
                angle = angles[i]
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                close_points.append(Point(x, y))
        
        if not close_points:
            return obstacles
        
        # Cluster close points to identify obstacles
        clusters = self._cluster_points(close_points)
        
        # Convert clusters to obstacles
        for cluster in clusters:
            if len(cluster) >= 3:  # Minimum points to form an obstacle
                obstacle = self._cluster_to_obstacle(cluster)
                obstacles.append(obstacle)
        
        # Update obstacle history
        self.obstacle_history.append(obstacles)
        if len(self.obstacle_history) > 10:  # Keep last 10 scans
            self.obstacle_history.pop(0)
        
        # Track moving obstacles
        self._track_moving_obstacles(obstacles)
        
        self.detected_obstacles = obstacles
        return obstacles
    
    def _cluster_points(self, points: List[Point]) -> List[List[Point]]:
        """Cluster points that are close to each other"""
        if not points:
            return []
        
        clusters = []
        visited = set()
        
        for i, point in enumerate(points):
            if i in visited:
                continue
            
            # Start new cluster
            cluster = [point]
            visited.add(i)
            
            # Find all points close to this one
            for j, other_point in enumerate(points):
                if j in visited:
                    continue
                
                if point.distance_to(other_point) < self.cluster_threshold:
                    cluster.append(other_point)
                    visited.add(j)
            
            clusters.append(cluster)
        
        return clusters
    
    def _cluster_to_obstacle(self, cluster: List[Point]) -> Obstacle:
        """Convert a cluster of points to an obstacle"""
        # Calculate centroid
        centroid_x = sum(p.x for p in cluster) / len(cluster)
        centroid_y = sum(p.y for p in cluster) / len(cluster)
        position = Point(centroid_x, centroid_y)
        
        # Calculate radius (distance from centroid to farthest point)
        radius = max(p.distance_to(position) for p in cluster)
        
        # Determine obstacle type based on history
        obstacle_type = self._classify_obstacle_type(position, radius)
        
        return Obstacle(
            position=position,
            radius=radius,
            confidence=0.8,  # Base confidence
            obstacle_type=obstacle_type,
            timestamp=time.time()
        )
    
    def _classify_obstacle_type(self, position: Point, radius: float) -> str:
        """Classify obstacle as static or dynamic"""
        if len(self.obstacle_history) < 2:
            return 'unknown'
        
        # Check if obstacle position has changed significantly
        for prev_obstacles in self.obstacle_history[-3:]:  # Check last 3 scans
            for prev_obs in prev_obstacles:
                if prev_obs.position.distance_to(position) < 0.5:  # Same obstacle
                    if prev_obs.position.distance_to(position) > 0.1:  # Moved
                        return 'dynamic'
                    else:
                        return 'static'
        
        return 'unknown'
    
    def _track_moving_obstacles(self, current_obstacles: List[Obstacle]):
        """Track moving obstacles and predict their movement"""
        # This is a simplified tracking implementation
        # In a real system, you'd use more sophisticated tracking algorithms
        pass

class NavigationPlanner:
    """Plans navigation path avoiding obstacles"""
    
    def __init__(self, robot_radius: float = 0.5, safety_margin: float = 0.3):
        self.robot_radius = robot_radius
        self.safety_margin = safety_margin
        self.goal_position: Optional[Point] = None
        self.current_position = Point(0, 0)
        self.current_heading = 0.0  # radians
        
    def set_goal(self, goal_x: float, goal_y: float):
        """Set navigation goal"""
        self.goal_position = Point(goal_x, goal_y)
        logger.info(f"Navigation goal set to ({goal_x}, {goal_y})")
    
    def plan_path(self, obstacles: List[Obstacle]) -> NavigationCommand:
        """Plan navigation path avoiding obstacles"""
        if not self.goal_position:
            return NavigationCommand(0, 0, 'stop', 0.0, time.time())
        
        # Calculate desired direction to goal
        desired_angle = self.current_position.angle_to(self.goal_position)
        angle_diff = desired_angle - self.current_heading
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > np.pi:
            angle_diff -= 2*np.pi
        while angle_diff < -np.pi:
            angle_diff += 2*np.pi
        
        # Check for obstacles in the path
        if self._is_path_clear(obstacles, desired_angle):
            # Path is clear, move toward goal
            linear_vel = 0.5  # m/s
            angular_vel = 0.5 * angle_diff  # Proportional control
            
            direction = 'forward'
            if abs(angle_diff) > np.pi/4:  # Large angle difference
                direction = 'left' if angle_diff > 0 else 'right'
                linear_vel = 0.1  # Slow forward movement while turning
            
        else:
            # Obstacle detected, find alternative path
            return self._find_alternative_path(obstacles)
        
        # Limit velocities
        linear_vel = np.clip(linear_vel, -0.5, 0.5)
        angular_vel = np.clip(angular_vel, -1.0, 1.0)
        
        confidence = 0.8 if self._is_path_clear(obstacles, desired_angle) else 0.6
        
        return NavigationCommand(
            linear_velocity=linear_vel,
            angular_velocity=angular_vel,
            direction=direction,
            confidence=confidence,
            timestamp=time.time()
        )
    
    def _is_path_clear(self, obstacles: List[Obstacle], desired_angle: float) -> bool:
        """Check if path in desired direction is clear of obstacles"""
        look_ahead_distance = 2.0  # meters
        
        # Check for obstacles in the path
        for obstacle in obstacles:
            # Calculate distance from robot to obstacle along desired path
            dx = obstacle.position.x - self.current_position.x
            dy = obstacle.position.y - self.current_position.y
            
            # Project obstacle position onto desired path
            path_vector = np.array([math.cos(desired_angle), math.sin(desired_angle)])
            obstacle_vector = np.array([dx, dy])
            
            # Distance along path
            distance_along_path = np.dot(obstacle_vector, path_vector)
            
            # Distance perpendicular to path
            distance_perpendicular = np.linalg.norm(obstacle_vector - distance_along_path * path_vector)
            
            # Check if obstacle is in the way
            if (0 < distance_along_path < look_ahead_distance and 
                distance_perpendicular < (self.robot_radius + obstacle.radius + self.safety_margin)):
                return False
        
        return True
    
    def _find_alternative_path(self, obstacles: List[Obstacle]) -> NavigationCommand:
        """Find alternative path when direct path is blocked"""
        # Simple obstacle avoidance: turn left or right
        # In a real system, you'd implement more sophisticated path planning
        
        # Find the direction with the most free space
        left_space = self._calculate_free_space(obstacles, -np.pi/2)
        right_space = self._calculate_free_space(obstacles, np.pi/2)
        
        if left_space > right_space:
            direction = 'left'
            angular_vel = -0.5
        else:
            direction = 'right'
            angular_vel = 0.5
        
        return NavigationCommand(
            linear_velocity=0.1,  # Slow forward movement
            angular_velocity=angular_vel,
            direction=direction,
            confidence=0.5,
            timestamp=time.time()
        )
    
    def _calculate_free_space(self, obstacles: List[Obstacle], angle: float) -> float:
        """Calculate free space in a given direction"""
        look_ahead_distance = 3.0
        min_distance = look_ahead_distance
        
        for obstacle in obstacles:
            dx = obstacle.position.x - self.current_position.x
            dy = obstacle.position.y - self.current_position.y
            
            # Calculate distance to obstacle in the given direction
            path_vector = np.array([math.cos(angle), math.sin(angle)])
            obstacle_vector = np.array([dx, dy])
            
            distance_along_path = np.dot(obstacle_vector, path_vector)
            distance_perpendicular = np.linalg.norm(obstacle_vector - distance_along_path * path_vector)
            
            if (distance_along_path > 0 and 
                distance_perpendicular < (self.robot_radius + obstacle.radius + self.safety_margin)):
                min_distance = min(min_distance, distance_along_path)
        
        return min_distance
    
    def update_position(self, linear_velocity: float, angular_velocity: float, dt: float):
        """Update robot position based on movement"""
        # Simple kinematic model
        self.current_heading += angular_velocity * dt
        
        # Normalize heading to [0, 2pi]
        while self.current_heading > 2*np.pi:
            self.current_heading -= 2*np.pi
        while self.current_heading < 0:
            self.current_heading += 2*np.pi
        
        # Update position
        self.current_position.x += linear_velocity * math.cos(self.current_heading) * dt
        self.current_position.y += linear_velocity * math.sin(self.current_heading) * dt

class IRISNavigationSystem:
    """Main navigation system for IRIS robot"""
    
    def __init__(self, use_simulation: bool = True):
        self.use_simulation = use_simulation
        
        # Initialize components
        if use_simulation:
            self.lidar = LIDARSimulator()
        else:
            # In real implementation, initialize actual LIDAR hardware
            self.lidar = None
            logger.warning("Real LIDAR not implemented - using simulation")
        
        self.obstacle_detector = ObstacleDetector()
        self.navigation_planner = NavigationPlanner()
        
        # Navigation state
        self.is_navigating = False
        self.navigation_thread = None
        self.command_queue = queue.Queue()
        
        # Performance metrics
        self.metrics = {
            'total_distance': 0.0,
            'obstacles_detected': 0,
            'navigation_time': 0.0,
            'collisions_avoided': 0
        }
        
        logger.info("IRIS Navigation System initialized")
    
    def start_navigation(self, goal_x: float, goal_y: float):
        """Start autonomous navigation to goal"""
        if self.is_navigating:
            logger.warning("Navigation already in progress")
            return
        
        self.navigation_planner.set_goal(goal_x, goal_y)
        self.is_navigating = True
        
        # Start navigation thread
        self.navigation_thread = threading.Thread(target=self._navigation_loop)
        self.navigation_thread.daemon = True
        self.navigation_thread.start()
        
        logger.info(f"Navigation started to goal ({goal_x}, {goal_y})")
    
    def stop_navigation(self):
        """Stop autonomous navigation"""
        self.is_navigating = False
        if self.navigation_thread:
            self.navigation_thread.join(timeout=1.0)
        
        # Send stop command
        stop_cmd = NavigationCommand(0, 0, 'stop', 1.0, time.time())
        self.command_queue.put(stop_cmd)
        
        logger.info("Navigation stopped")
    
    def _navigation_loop(self):
        """Main navigation loop"""
        start_time = time.time()
        last_update = start_time
        
        while self.is_navigating:
            try:
                current_time = time.time()
                dt = current_time - last_update
                
                # Get LIDAR data
                if self.use_simulation:
                    distances = self.lidar.get_scan_data()
                    angles = self.lidar.angles
                else:
                    # Real LIDAR implementation would go here
                    continue
                
                # Detect obstacles
                obstacles = self.obstacle_detector.detect_obstacles(distances, angles)
                
                # Plan navigation
                command = self.navigation_planner.plan_path(obstacles)
                
                # Update robot position
                self.navigation_planner.update_position(
                    command.linear_velocity, 
                    command.angular_velocity, 
                    dt
                )
                
                # Send command
                self.command_queue.put(command)
                
                # Update metrics
                self._update_metrics(command, obstacles, dt)
                
                # Check if goal reached
                if self._is_goal_reached():
                    logger.info("Navigation goal reached!")
                    self.is_navigating = False
                    break
                
                last_update = current_time
                time.sleep(0.1)  # 10 Hz update rate
                
            except Exception as e:
                logger.error(f"Error in navigation loop: {e}")
                break
        
        self.metrics['navigation_time'] = time.time() - start_time
        logger.info("Navigation loop ended")
    
    def _is_goal_reached(self) -> bool:
        """Check if navigation goal has been reached"""
        if not self.navigation_planner.goal_position:
            return False
        
        distance = self.navigation_planner.current_position.distance_to(
            self.navigation_planner.goal_position
        )
        
        return distance < 0.5  # 0.5 meter tolerance
    
    def _update_metrics(self, command: NavigationCommand, obstacles: List[Obstacle], dt: float):
        """Update performance metrics"""
        # Update total distance
        self.metrics['total_distance'] += abs(command.linear_velocity) * dt
        
        # Update obstacles detected
        self.metrics['obstacles_detected'] = len(obstacles)
        
        # Update collisions avoided (simplified)
        if command.direction in ['left', 'right'] and len(obstacles) > 0:
            self.metrics['collisions_avoided'] += 1
    
    def get_navigation_status(self) -> Dict:
        """Get current navigation status"""
        return {
            'is_navigating': self.is_navigating,
            'current_position': {
                'x': self.navigation_planner.current_position.x,
                'y': self.navigation_planner.current_position.y,
                'heading': math.degrees(self.navigation_planner.current_heading)
            },
            'goal_position': {
                'x': self.navigation_planner.goal_position.x if self.navigation_planner.goal_position else None,
                'y': self.navigation_planner.goal_position.y if self.navigation_planner.goal_position else None
            },
            'metrics': self.metrics.copy()
        }
    
    def visualize_navigation(self, obstacles: List[Obstacle], command: NavigationCommand):
        """Create visualization of current navigation state"""
        try:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
            
            # Plot 1: Robot position and goal
            ax1.set_xlim(-5, 5)
            ax1.set_ylim(-5, 5)
            ax1.grid(True)
            ax1.set_aspect('equal')
            
            # Plot robot
            robot_pos = self.navigation_planner.current_position
            ax1.plot(robot_pos.x, robot_pos.y, 'bo', markersize=10, label='Robot')
            
            # Plot robot heading
            heading_x = robot_pos.x + math.cos(self.navigation_planner.current_heading)
            heading_y = robot_pos.y + math.sin(self.navigation_planner.current_heading)
            ax1.plot([robot_pos.x, heading_x], [robot_pos.y, heading_y], 'b-', linewidth=2)
            
            # Plot goal
            if self.navigation_planner.goal_position:
                goal_pos = self.navigation_planner.goal_position
                ax1.plot(goal_pos.x, goal_pos.y, 'g*', markersize=15, label='Goal')
            
            # Plot obstacles
            for obstacle in obstacles:
                circle = plt.Circle(
                    (obstacle.position.x, obstacle.position.y), 
                    obstacle.radius, 
                    color='red', alpha=0.6
                )
                ax1.add_patch(circle)
            
            ax1.set_title('Navigation Map')
            ax1.legend()
            
            # Plot 2: LIDAR scan data
            if self.use_simulation:
                distances = self.lidar.get_scan_data()
                angles = self.lidar.angles
                
                # Convert to Cartesian coordinates
                x = distances * np.cos(angles)
                y = distances * np.sin(angles)
                
                ax2.scatter(x, y, c=distances, cmap='viridis', alpha=0.7)
                ax2.set_xlim(-12, 12)
                ax2.set_ylim(-12, 12)
                ax2.set_aspect('equal')
                ax2.set_title('LIDAR Scan Data')
                ax2.set_xlabel('X (m)')
                ax2.set_ylabel('Y (m)')
            
            plt.tight_layout()
            plt.show()
            
        except Exception as e:
            logger.error(f"Error creating visualization: {e}")

def main():
    """Main function for testing the navigation system"""
    parser = argparse.ArgumentParser(description='IRIS Navigation System')
    parser.add_argument('--goal-x', type=float, default=5.0, help='Goal X coordinate')
    parser.add_argument('--goal-y', type=float, default=3.0, help='Goal Y coordinate')
    parser.add_argument('--simulation', action='store_true', help='Use simulation mode')
    parser.add_argument('--visualize', action='store_true', help='Show visualization')
    
    args = parser.parse_args()
    
    try:
        # Initialize navigation system
        nav_system = IRISNavigationSystem(use_simulation=args.simulation)
        
        # Start navigation
        nav_system.start_navigation(args.goal_x, args.goal_y)
        
        # Monitor navigation
        try:
            while nav_system.is_navigating:
                status = nav_system.get_navigation_status()
                print(f"\rPosition: ({status['current_position']['x']:.2f}, {status['current_position']['y']:.2f}) "
                      f"Heading: {status['current_position']['heading']:.1f}° "
                      f"Distance: {status['metrics']['total_distance']:.2f}m", end='')
                
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\nNavigation interrupted by user")
            nav_system.stop_navigation()
        
        # Final status
        final_status = nav_system.get_navigation_status()
        print(f"\n\nNavigation completed!")
        print(f"Final position: ({final_status['current_position']['x']:.2f}, {final_status['current_position']['y']:.2f})")
        print(f"Total distance traveled: {final_status['metrics']['total_distance']:.2f}m")
        print(f"Obstacles detected: {final_status['metrics']['obstacles_detected']}")
        print(f"Collisions avoided: {final_status['metrics']['collisions_avoided']}")
        print(f"Navigation time: {final_status['metrics']['navigation_time']:.1f}s")
        
    except Exception as e:
        logger.error(f"Error in main execution: {e}")
        return 1
    
    return 0

if __name__ == "__main__":
    exit(main())

