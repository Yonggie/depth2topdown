import omnigibson.lazy as lazy
import cv2
from helper import depth_to_point_cloud, transform_point_cloud

from omnigibson.utils.ui_utils import KeyboardEventHandler

import torch
import omnigibson as og
from omnigibson.sensors import VisionSensor
from config import cfg, move, rotation, start_pos
import numpy as np

ENV_X_MIN = -30
ENV_X_MAX = 30
ENV_Y_MIN = -30
ENV_Y_MAX = 30
BEV_RESOLUTION = 0.1 # Meters per pixel
BEV_WIDTH = int((ENV_X_MAX - ENV_X_MIN) / BEV_RESOLUTION)
BEV_HEIGHT = int((ENV_Y_MAX - ENV_Y_MIN) / BEV_RESOLUTION)


# Shared variables for images
bev_map = torch.zeros((BEV_HEIGHT, BEV_WIDTH), dtype=torch.uint8)  # 0: free, 255: occupied
print('initial bev map shape', bev_map.shape)
current_rgb_image = None
current_depth_image = None
camera_path = []


env = og.Environment(cfg)

cam_mover = VisionSensor(
        prim_path="/World/viewer_camera",  # prim path
        # relative_prim_path="/World/scene_0/controllable_fetch_robot_flikke/eyes/Camera",
        name="my_vision_sensor",  # sensor name
        modalities=["rgb","depth_linear"],  # sensor mode
        enabled=True,  # enale sensor
        image_height=480,  # 
        image_width=640,  # 
        focal_length=17,  
        clipping_range=(0.01, 1000000.0),  # vision distance
    )
cam_mover.initialize()
og.sim.enable_viewer_camera_teleoperation()

KeyboardEventHandler.initialize()
cam_pos=[-0.90102798, 14.00433922,  1.55399889]
cam_mover.set_position(cam_pos)


def update_all():
    new_position, new_orientation = cam_mover.get_position_orientation()
    camera_path.append(new_position)

    # update the environment
    for _ in range(4): 
        env.step(np.array([0,0]))  

    obs = cam_mover.get_obs()[0]
    rgb = obs["rgb"][..., :3][..., ::-1]
    depth = obs["depth_linear"]


    current_rgb_image = rgb.copy()
    current_depth_image = depth.copy()
    
    # count += 1
    camera_points = depth_to_point_cloud(current_depth_image)
    world_points = transform_point_cloud(points=camera_points, pose = new_position, orientation=new_orientation)
    update_bev_map(world_points)
    
    
    view_bev()


def update_bev_map(points_world):
    global bev_map
    # Filter points with height (z) greater than 0.5 meters
    height_threshold = 0.5
    points_above_threshold = points_world[(points_world[:, 1] > height_threshold) & (points_world[:, 1] < 2)]

    # Convert world coordinates to BEV map indices
    x_indices = ((points_above_threshold[:, 0] - ENV_X_MIN) / BEV_RESOLUTION)
    y_indices = ((points_above_threshold[:, 2] - ENV_Y_MIN) / BEV_RESOLUTION)
    
    # Ensure indices are within the map boundaries
    valid_indices = (
        (x_indices >= 0) & (x_indices < BEV_WIDTH) &
        (y_indices >= 0) & (y_indices < BEV_HEIGHT)
    )
    x_indices = x_indices[valid_indices].long()
    y_indices = y_indices[valid_indices].long()

    # Update the BEV map
    
    bev_map[y_indices, x_indices] = 255  # Mark as occupied


def view_bev():
    bev_map_image = bev_map.cpu().numpy()
    print('bev_map shape', bev_map.shape)
    # Optionally, convert to a color image for better visualization
    bev_map_color = cv2.cvtColor(bev_map_image, cv2.COLOR_GRAY2BGR)
    # You might want to flip or rotate the image to match coordinate conventions
    bev_map_color = cv2.flip(bev_map_color, 0)  # Flip vertically if needed

    _, encoded_image = cv2.imencode('.png', bev_map_color)
    print('bev encoded map shape', encoded_image.shape)
    # encoded_image.reshape(BEV_HEIGHT,BEV_WIDTH)
    cv2.imwrite('bev_map.png',bev_map_image.astype(np.uint8))
    cv2.imwrite('bev_encoded_map.png',encoded_image.astype(np.uint8))
    

    return encoded_image

KeyboardEventHandler.add_keyboard_callback(lazy.carb.input.KeyboardInput.A,update_all)
KeyboardEventHandler.add_keyboard_callback(lazy.carb.input.KeyboardInput.W,update_all)
KeyboardEventHandler.add_keyboard_callback(lazy.carb.input.KeyboardInput.S,update_all)
KeyboardEventHandler.add_keyboard_callback(lazy.carb.input.KeyboardInput.D,update_all)


while True:
    env.step(np.array([0,0])) 