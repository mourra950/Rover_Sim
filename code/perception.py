import numpy as np
import cv2
import scipy
white_mask=np.ones((200, 200), dtype=float)
clipping_num=100
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, above_thresh,below_thresh=(600,600,600)):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh_result = (img[:,:,0] > above_thresh[0]) \
                & (img[:,:,1] > above_thresh[1]) \
                & (img[:,:,2] > above_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh_result] = 1

  
    below_thresh_result = (img[:,:,0] > below_thresh[0]) \
                & (img[:,:,1] > below_thresh[1]) \
                & (img[:,:,2] > below_thresh[2])
    # Index the array of zeros with the boolean array and set to 0
    color_select[below_thresh_result] = 0   

    
    # Return the binary image
    return color_select
    
# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
        
    return warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(Rover):
    #Init some variable that will be used during the perception step
    dst_size= 6
    bottom_offset= 6
    image= Rover.img
    # Perform perception steps to update Rover()
    # TODO: 
    # NOTE: camera image is coming to you in Rover.img
    
    # 1) Define source and destination points for perspective transform
    #numbers are approximated from a test image with grid
    source = np.float32([[14, 140],
                     [300, 140],
                     [200, 95],
                     [120, 95]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                ])
    
    
    
    
    
    # 2) Apply perspective transform
    
    warped = perspect_transform(image, source, destination)
    
    

    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    #this values derived using the picker tool in photoshop to get the lowest dark color in accepted images
    Terrain_threshold=(150, 150, 150)
    Rock_threshhold=(120,120,-1)
    Rock_threshhold1=(-1,-1,60)
    
    #color threshold for all terrain and rocks
    ###################################################
    terrain_img=color_thresh(warped,Terrain_threshold)
    #threshed1 = color_thresh1(image,Rock_threshhold1)
    #threshed = color_thresh(image,Rock_threshhold)
    ###################################################
    #to get rock
    #threshed_rock = threshed1-threshed
    #threshed_rock=scipy.ndimage.binary_erosion(threshed_rock, structure=np.ones((3,3))).astype(threshed_rock.dtype)
    threshed_rock =color_thresh(warped,Rock_threshhold,Rock_threshhold1)
    # clip the far away results as they are not as accurate as i need
    #################################################
    #obstacle 
    white = cv2.bitwise_not(np.zeros_like(image)) #white image 
    warped_white = perspect_transform(white, source, destination)# warping our mask 
    warped_white_threshed = color_thresh(warped_white,(1,1,1))#thresholding not needed
    not_terrain = cv2.bitwise_not(terrain_img) #invert the terrain image
    obstacle = cv2.bitwise_and(warped_white_threshed,not_terrain)#applying the mask to the inverted image
    #obstacle[0:80]= 0
    #######################################################################
    terrain_img[0:clipping_num]=0
    obstacle[0:clipping_num]= 0
    threshed_rock[0:clipping_num]=0
    # 4) Update Rover.vision_image (this will be displayed on left side of screen)
        # Example: Rover.vision_image[:,:,0] = obstacle color-thresholded binary image
        #          Rover.vision_image[:,:,1] = rock_sample color-thresholded binary image
        #          Rover.vision_image[:,:,2] = navigable terrain color-thresholded binary image
    Rover.vision_image[:,:,0] = obstacle*255
    Rover.vision_image[:,:,1] = threshed_rock*255
    Rover.vision_image[:,:,2] = terrain_img*255
    
    # 5) Convert map image pixel values to rover-centric coords
    x_pixel_rover, y_pixel_rover=rover_coords(terrain_img)  #terrain rover
    x_pixel_obstacle,y_pixel_obstacle = rover_coords(obstacle) #obstacle rover
    x_pixel_rock,y_pixel_rock = rover_coords(threshed_rock)
    # 6) Convert rover-centric pixel values to world coordinates
    navigable_x_world,navigable_y_world =pix_to_world(x_pixel_rover,y_pixel_rover,Rover.pos[0],Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0],10)
    obstacle_x_world,obstacle_navigable_y_world = pix_to_world(x_pixel_obstacle,y_pixel_obstacle,Rover.pos[0],Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0],10)
    rock_x_world,rock_y_world =pix_to_world(x_pixel_rock,y_pixel_rock,Rover.pos[0],Rover.pos[1],Rover.yaw,Rover.worldmap.shape[0],10)
    # 7) Update Rover worldmap (to be displayed on right side of screen)
        # Example: Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          Rover.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 1
    if (Rover.roll >=0 and Rover.roll <=0.2) or (Rover.roll <=360 and Rover.roll >=359.7):
        Rover.worldmap[obstacle_navigable_y_world, obstacle_x_world, 0] += 50
        Rover.worldmap[rock_y_world, rock_x_world, 1] += 50
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 50 
    #byzwd weight of blue 3ashan yban aktr l2n el background akhdr f el blue yzeed w yghaty 3ala akhdar
    
    # 8) Convert rover-centric pixel positions to polar coordinates
    # Update Rover pixel distances and angles
        # Rover.nav_dists = rover_centric_pixel_distances
        # Rover.nav_angles = rover_centric_angles
    dist, angles=to_polar_coords(x_pixel_rover, y_pixel_rover)
    
    Rover.nav_dists = dist
    Rover.nav_angles= angles
    
 
    
    
    return Rover