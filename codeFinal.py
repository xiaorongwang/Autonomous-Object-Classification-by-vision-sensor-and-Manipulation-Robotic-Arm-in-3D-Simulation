import cv2
import numpy as np
import matplotlib.pyplot as plt
import coppeliasim_zmqremoteapi_client as zmq

import ecse275_vision_utils as util  # Ensure the module exists and the path is correct


# Camera parameters
f = 0.02  # Camera focal length in meters
pixels_per_inch = 560.0165995731867
pixels_per_meter = pixels_per_inch * 39.3701
vision_mode = "RGB"  # Vision mode

# Known height of the red module (in meters)
known_red_height = 0.4725  # Adjust based on actual values

def getZ(samplewidth, sampleheight, imagewidth):
    z = (samplewidth / imagewidth) * sampleheight
    return z

# Connect to CoppeliaSim
client = zmq.RemoteAPIClient()
sim = client.getObject('sim')
drop_target1 = sim.getObject('/DropTarget1')
drop_target2 = sim.getObject('/DropTarget2')
drop_target3 = sim.getObject('/DropTarget3')
camera = sim.getObject("/Vision_sensor")

# Define obstacles
obs1 = sim.getObject('/Obs1')
obs2 = sim.getObject('/Obs2')
obs3 = sim.getObject('/Obs3')
obs4 = sim.getObject('/Obs4')

# --- Obstacle avoidance setup ---
def setup_obstacle_avoidance(sim, target_handle, goal_handle, obstacle_handles):
    """
    Sets up the OMPL task for obstacle avoidance for the target object.
    """
    # Create OMPL task
    task = sim.ompl.createTask('obstacleAvoidance')

    # Expand the state space boundary to encourage the robot to find a broader path around obstacles
    state_space = sim.ompl.createStateSpace('6d', sim.ompl.StateSpaceType.pose3d, target_handle, [-2, -1.0, 0], [2, 1.0, 2], 1)
    sim.ompl.setStateSpace(task, [state_space])
    sim.ompl.setAlgorithm(task, sim.ompl.Algorithm.RRTConnect)

    # Set collision pairs with an additional buffer around obstacles to make the path more conservative
    collision_pairs = obstacle_handles + [sim.handle_all]
    sim.ompl.setCollisionPairs(task, collision_pairs)

    # Define start and goal poses
    start_pose = sim.getObjectPose(target_handle, -1)
    goal_pose = sim.getObjectPose(goal_handle, -1)

    sim.ompl.setStartState(task, start_pose)
    sim.ompl.setGoalState(task, goal_pose)

    return task

def execute_obstacle_avoidance(sim, task, target_handle):
    """
    Executes the planned path for the target object to avoid obstacles and reach the goal.
    """
    sim.ompl.setup(task)
    result, path = sim.ompl.compute(task, 20, -1, 200)

    if not result or not path:
        raise Exception("Path planning failed.")

    # Execute the path
    path_points = np.array(path).reshape(-1, 7)  # Reshape into [position (x, y, z), quaternion (qx, qy, qz, qw)]
    for point in path_points:
        position = point[:3]
        quaternion = point[3:]
        sim.setObjectPosition(target_handle, -1, position.tolist())
        sim.setObjectQuaternion(target_handle, -1, quaternion.tolist())
        sim.stepSimulation()

# Get the camera's transformation matrix relative to the world
T_cam_world = np.array(sim.getObjectMatrix(camera, -1)).reshape(3, 4)

# Acquire image data from the vision sensor
image, resolution = sim.getVisionSensorImg(camera, 0)
print(f"Resolution: {resolution}")
image = np.array(bytearray(image), dtype='uint8').reshape(resolution[0], resolution[1], 3)

# Convert and visualize the image
image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
image = np.flip(image, axis=1)  # Flip the image horizontally
plt.figure(figsize=(8, 6))
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))  # Display the image in RGB
plt.title("Original Image")
plt.axis('off')
plt.show()

# Apply morphological operations to reduce noise
kernel = np.ones((3, 3), np.uint8)  # Adjust kernel size
image_blur = cv2.GaussianBlur(image, (3, 3), 0)  # Apply Gaussian blur
image_morph = cv2.morphologyEx(image_blur, cv2.MORPH_OPEN, kernel)

# Convert the image to HSV color space
hsv = cv2.cvtColor(image_morph, cv2.COLOR_BGR2HSV)

# Define HSV range for red and blue colors
lower_red1 = np.array([0, 100, 150])
upper_red1 = np.array([10, 255, 255])
lower_red2 = np.array([170, 70, 50])
upper_red2 = np.array([180, 255, 255])
lower_blue = np.array([120, 220, 175])
upper_blue = np.array([140, 255, 255])

# Create masks for red and blue colors
mask_red = cv2.inRange(hsv, lower_red1, upper_red1)
mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

# Visualize color masks
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.title("Red Mask")
plt.imshow(mask_red, cmap='gray')
plt.axis('off')
plt.subplot(1, 2, 2)
plt.title("Blue Mask")
plt.imshow(mask_blue, cmap='gray')
plt.axis('off')
plt.show()

# Detect contours for red and blue masks
contours_red, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

print(f"Detected {len(contours_red)} red contours and {len(contours_blue)} blue contours.")

# Initialize dictionary to store centroids, edge lengths, and heights
shape_centroids = {
    'Red_Square': [], 'Red_Rectangle': [], 'Red_Circle': [],
    'Blue_Square': [], 'Blue_Rectangle': [], 'Blue_Circle': []
}
scale = None

def process_contours(contours, color_label, image_draw):
    global scale
    for contour in contours:
        #ignore the small contours to prevent subtle variations in readings
        area = cv2.contourArea(contour)
        if area < 50:
            continue


        #calculate vertices and circularity by cv2
        x, y, w, h = cv2.boundingRect(contour)
        examplewidth = float(w) if w < h else float(h)
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            continue
        epsilon = 0.03 * perimeter
        approx = cv2.approxPolyDP(contour, epsilon, True)

        num_vertices = len(approx)
        circularity = 4 * np.pi * (area / (perimeter * perimeter)) if perimeter != 0 else 0
        aspect_ratio = float(w) / h if h != 0 else 0

        print(f"{color_label} Contour: Vertices={num_vertices}, Circularity={circularity:.2f}, Aspect Ratio={aspect_ratio:.2f}")


        #identify the shape based on circularity and vertices
        if num_vertices == 4 and 0.5 <= circularity <= 0.9:
            angles = []
            for i in range(4):
                p1 = approx[i][0]
                p2 = approx[(i + 1) % 4][0]
                p3 = approx[(i + 2) % 4][0]

                v1 = p1 - p2
                v2 = p3 - p2
                dot = np.dot(v1, v2)
                norm = (np.linalg.norm(v1) * np.linalg.norm(v2)) + 1e-6
                angle = np.degrees(np.arccos(dot / norm))
                angles.append(angle)

            if all(75 <= angle <= 105 for angle in angles):
                shape = "Square" if 0.8 <= aspect_ratio <= 1.2 else "Rectangle"
            else:
                shape = "Unknown"
        elif circularity >= 0.7 and num_vertices > 5:
            shape = "Circle"
        else:
            shape = "Unknown"

        if shape == "Unknown":
            continue


        #calculate centroid using cv2
        M = cv2.moments(contour)
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx = x + w // 2
            cy = y + h // 2

        if color_label == "Red":
            scale = examplewidth



        #using the red cuboid as reference to calculate the blue blocks
        blue_height = getZ(scale, known_red_height, examplewidth) if color_label == "Blue" else None
        key = f"{color_label}_{shape}"
        shape_centroids[key].append((cx, cy, blue_height if color_label == "Blue" else known_red_height, examplewidth))

        contour_color = (0, 0, 255) if color_label == "Red" else (255, 0, 0)
        cv2.drawContours(image_draw, [contour], -1, contour_color, 2)
        label_text = f"{color_label} {shape} H={blue_height:.3f}m" if color_label == "Blue" and blue_height is not None else f"{color_label} {shape}"
        cv2.putText(image_draw, label_text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, contour_color, 2)

    return image_draw

# Create a copy of the image for drawing
image_draw = image.copy()
image_draw = process_contours(contours_red, "Red", image_draw)
image_draw = process_contours(contours_blue, "Blue", image_draw)

# Show the processed image
plt.figure(figsize=(12, 8))
plt.imshow(cv2.cvtColor(image_draw, cv2.COLOR_BGR2RGB))
plt.title('Detected Colored Shapes with Heights')
plt.axis('off')
plt.show()

# Print shape centroids, edge lengths, and heights
for key, centroids in shape_centroids.items():
    print(f"{key}: {centroids}")

#change pixel postion to camera frame postion function
def PixToPosition(shape_centroid, resolution, f, pixels_per_meter, zz):
    x_pos = ((shape_centroid[0] - (resolution[1] / 2)) * zz) / (f * pixels_per_meter)
    y_pos = ((shape_centroid[1] - (resolution[0] / 2)) * zz) / (f * pixels_per_meter)
    return np.array((x_pos, y_pos, zz))

# Convert centroids to world coordinates
pos_world_list = []
for key, centroids in shape_centroids.items():
    color, shape = key.split('_')
    if color == 'Red':
        continue
    for centroid in centroids:
        cx, cy, height, edge_length = centroid
        position = PixToPosition((cx, cy), resolution, f, pixels_per_meter, height)
        print(f"This {color} {shape} at pos: {position}, Edge Length: {edge_length}, Height: {height}m")
        position_world = util.hand_eye_transform(position, T_cam_world)
        pos_world_list.append((color, shape, position_world, edge_length, height))
        print(f"Computed World Position for {color} {shape}: {position_world}")

# Define drop targets
drop_targets = {
    "Square": drop_target1,
    "Rectangle": drop_target2,
    "Circle": drop_target3
}

# Move the robot to pick and drop the detected objects
for item in pos_world_list:
    color, shape, pos_world, edge_length, height = item
    print(f"Moving to pick up {color} {shape} at position: {pos_world}, Edge Length: {edge_length}, Height: {height}m")

    # --- Add obstacle avoidance ---
    try:
        # Define obstacle avoidance target and goal
        target_handle = sim.getObject('/Target')  # Assume a target object to pick (replace with actual)
        goal_handle = drop_targets[shape]
        obstacle_handles = [obs1, obs2, obs3,obs4]

        # Set up obstacle avoidance with more conservative path planning
        task = setup_obstacle_avoidance(sim, target_handle, goal_handle, obstacle_handles)

        # Execute obstacle avoidance
        execute_obstacle_avoidance(sim, task, target_handle)

    except Exception as e:
        print(f"Error during obstacle avoidance: {e}")

    # Continue with pick and place
    util.move_to(sim, list(pos_world))
    util.toggle_gripper(sim)

    if shape in drop_targets:
        drop_target = drop_targets[shape]
        drop_target_pose = sim.getObjectPose(drop_target, -1)
        drop_target_pose[2] += 0.035  # Adjust height
        print(f"Moving to drop {color} {shape} at target position: {drop_target_pose}")
        util.move_to(sim, drop_target_pose)
        util.toggle_gripper(sim)

print("Task completed successfully.")
