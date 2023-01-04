import numpy as np
import os
def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]
# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):
    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    # Example:
    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:
        # Check for Rover.mode status
       # print(len(Rover.navstop_angles))
        if Rover.mode == 'forward': 
            Rover.frames_stop=0
            # Check the extent of navigable terrain
            if len(Rover.navstop_angles) >= Rover.stop_forward:  
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                else: # Else coast
                    Rover.throttle = 0
                Rover.brake = 0
                # Set steering to average angle clipped to the range +/- 15
                
                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-15,15)
            # If there's a lack of navigable terrain pixels then go to 'stop' mode
            elif len(Rover.navstop_angles) < Rover.stop_forward:
                    # Set mode to "stop" and hit the brakes!
                    Rover.throttle = 0
                    # Set brake to stored brake value
                    Rover.brake = Rover.brake_set
                    Rover.steer = 0
                    Rover.mode = 'stop'
        # If we're already in "stop" mode then make different decisions
        elif Rover.mode == 'stop':
            # If we're in stop mode but still moving keep braking
            if Rover.vel > 0.2:
                Rover.throttle = 0
                Rover.brake = Rover.brake_set
                Rover.steer = 0
            # If we're not moving (vel < 0.2) then do something else
            elif Rover.vel <= 0.2 and Rover.frames_stop>100:
                #print(len(Rover.navstop_angles))
                # Now we're stopped and we have vision data to see if there's a path forward
                if len(Rover.navstop_angles) < Rover.go_forward:
                    Rover.throttle = 0
                    # Release the brake to allow turning
                    Rover.brake = 0
                    
                    Rover.steer = -4 # Could be more clever here about which way to turn
                # If we're stopped but see sufficient navigable terrain in front then go!
                if len(Rover.navstop_angles) >= Rover.go_forward:
                    # Set throttle back to stored value
                    Rover.throttle = Rover.throttle_set
                    # Release the brake
                    Rover.brake = 0
                    # Set steer to mean angle
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    Rover.mode = 'forward'
            Rover.frames_stop+=1
        elif Rover.mode == 'Rock_in_sight':
            Rover.frames_stop=0
            steerterrain=0
            dist_to_rock = min(Rover.navrock_dists) 
            nav_power_steering=0
            if 11<dist_to_rock<=18:
                nav_power_steering=5
            elif 18<dist_to_rock<=45:
                nav_power_steering=8
            elif 45<dist_to_rock:
                nav_power_steering=12
            if(Rover.nav_angles.any()):
                steerterrain=np.clip(np.mean(Rover.nav_angles * 180/np.pi),-nav_power_steering,nav_power_steering)
            dist_to_rock = min(Rover.navrock_dists) 
            #print(Rover.near_sample)
            if (Rover.near_sample==0) or (dist_to_rock>9):
                # Check the extent of navigable terrain
                # If mode is forward, navigable terrain looks good 
                # and velocity is below max, then throttle 
                if Rover.vel < Rover.max_vel:
                    # Set throttle value to throttle setting
                    Rover.throttle = Rover.throttle_set
                    Rover.brake = 0
                else: # Else coast
                    Rover.throttle = 0
                    Rover.brake = 1
                # Set steering to average angle clipped to the range +/- 15
                
                Rover.steer = np.clip(np.mean(Rover.navrock_angles * 180/np.pi),-15,15) +steerterrain

                
            else: #rock is in position to be picked use brakes to stop to be able to pick it
                    if Rover.vel > 0:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        #code for steering angle to move to pick rock and reach initial position
        """
        elif Rover.mode == 'Rock_in_sight':
            ##########################################################################
            #dividing this mode to different parts depending on how far the nearest part of the rock is compared to us to adjust the steering power and maximum speed
            ##########################################################################
            #by checking if the flag is raised or not the rover keep moving toward the rock
            if Rover.near_sample==0: 
                #store the nearest part of the located rock to the rover
                dist_to_rock = min(Rover.nav_dists) 
                print(max(Rover.nav_angles)* 180/np.pi)
                #if nearest part of the rock in distance between 35 to 55 pixels
                if 65>dist_to_rock > 45 :
                    if Rover.vel < 1:
                        Rover.brake = 0
                        Rover.throttle = Rover.throttle_set
                    else:
                        Rover.throttle = 0
                        Rover.brake = 1
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-10,6)
                #if nearest part of the rock in distance between 22 to 35 pixels
                elif dist_to_rock > 22:
                    if Rover.vel < 0.6:
                        Rover.throttle = 2
                        Rover.brake = 0 
                    else:
                        Rover.throttle = 0
                        Rover.brake = 4
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-12,8)
                #if nearest part of the rock in distance between 12 to 22 pixels
                elif dist_to_rock > 12:
                    if Rover.vel < 0.7:
                        # Set throttle value to throttle setting
                        Rover.throttle = Rover.throttle_set
                        Rover.brake = 0 
                    else: # Else coast
                        Rover.throttle = 0
                        Rover.brake = 4
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-14,10)
                #if nearest part of the rock in distance is less then 12 so the rock is pretty close
                elif dist_to_rock <12:
                    if Rover.vel >= 0.4:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                    else:
                        #check if the rock mean is in range between 20 and -20 in angles
                        if np.mean(Rover.nav_angles * 180/np.pi) <20 and np.mean(Rover.nav_angles * 180/np.pi) >-20 :
                            Rover.throttle=0.2
                            Rover.brake=0
                            Rover.steer=0 
                        else:
                            if Rover.vel <= 0.2:
                                Rover.brake = 0
                                Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi),-10,10)
                            else:
                                Rover.brake = Rover.brake_set
                                Rover.throttle=0
                                Rover.steer = 0
            else: #rock is in position to be picked use brakes to stop to be able to pick it
                    if Rover.vel > 0:
                        Rover.throttle = 0
                        Rover.brake = Rover.brake_set
                        Rover.steer = 0
                        #code for steering angle to move to pick rock and reach initial position
          """
        # Just to make the rover do something 
        # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0
        
    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.steer = 0
        Rover.brake = 0
        Rover.send_pickup = True
        Rover.mode == 'forward'
    
    if Rover.mapped_percentage >95 and Rover.samples_collected>=5:
        Rover.home_return_flag=1
      
    if Rover.home_return_flag==1:
        if Rover.initpoint[0]+3>Rover.pos[0]>Rover.initpoint[0]-3 and Rover.initpoint[1]+3>Rover.pos[1]>Rover.initpoint[1]-3:
            Rover.steer=0
            Rover.throttle=0
            Rover.brake=Rover.brake_set
            os.system('cls')
            print('You are Home welcome back champ')
    return Rover
