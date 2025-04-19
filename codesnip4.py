import lidar
import numpy as np

def detect_obstacles():
    # Get distance data from LIDAR sensor
    distances = lidar.get_distances()

    # Analyze the distance data to check for obstacles
    if np.min(distances) < 2.0:  # If any obstacle is within 2 meters
        return True
    return False

def navigate():
    if detect_obstacles():
        print("Obstacle detected! Re-routing...")
        # Code to avoid obstacle
        # For example, turn 90 degrees and continue
        turn(90)
    else:
        print("Path is clear, moving forward.")

# Continuously check for obstacles and navigate
while True:
    navigate()
    time.sleep(1)  # Check every second

