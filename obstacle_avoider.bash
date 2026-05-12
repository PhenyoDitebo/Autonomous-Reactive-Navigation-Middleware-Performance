#!/usr/bin/bash

# include the functions library
source ./robot_functions.bash

set_cmd_vel_linear () {
 ros2 param set /robot_interface cmd_vel_linear $1
 return 0
}

set_cmd_vel_angular () {
 ros2 param set /robot_interface cmd_vel_angular $1
 return 0
}

# naive obstacle avoider
echo "Running Naive Obstacle Avoider with Bash Script..."
echo "Press Ctrl+C to Terminate..."

set_cmd_vel_linear 0.000
set_cmd_vel_angular 0.000

# set obstacle avoidance distance threshold
threshold=0.600 #meters

# main while loop for naive obstacle avoider
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

while :
do
  # 1. GET THE FRONT RANGE
  ff_range=$(get_scan_front_ray_range)
  
  # if the sensor sees we have a large distance ahead, or detects nothing (computer failure)
  # assign it a large number we can use for calculation of the decision tree later.
  if [[ "$ff_range" == "inf" || -z "$ff_range" ]]; then ff_range=10.0; fi

  threshold=0.6 # this high because the simulation was really hard 
  # to fix without crashing into walls (same with real robot)

  # 2. CHECK IF FRONT IS FREE using simple bool logic
  # returns a boolean for 0 (obstacle ahead) or 1 (Nothing ahead)
  front_free=$(echo "$ff_range > $threshold" | bc -l)

  if [[ "$front_free" == "0" ]]; then
    echo "OBSTACLE! BRAKING!"
    set_cmd_vel_linear 0.000

    # case 1: front path blocked -> decide direction
    ll_range=$(get_scan_left_ray_range)
    rr_range=$(get_scan_right_ray_range)

    if (( $(echo "$ll_range > $rr_range" | bc -l) )); then
        echo "Turning left (more space on left)"
        set_cmd_vel_angular 0.942477 # about 0.3*pi rad/s or 54 deg/s (for lidar 'vision' to not blur)
        # 0.1 rad/s was too slow for my liking
    else
        echo "Turning right (more space on the right)"
        set_cmd_vel_angular -0.942477
    fi

    # 0.5 gives the robot time to move
    sleep 0.5 
    set_cmd_vel_angular 0.000

  else 
    # Case 2: path clear -> move forward
    dist_to_move=$(echo "$ff_range - $threshold" | bc -l)
    time_to_move=$(echo "$dist_to_move / 0.1500" | bc -l)
    time_to_move=$(echo "$time_to_move - 0.100" | bc -l)

    if (( $(echo "$time_to_move > 0" | bc -l) )); then
        set_cmd_vel_linear 0.15
        sleep 0.1

    else
        # might not be needed, but redundency is key
        # no slip ups. 
        echo "Approaching threshold limit... slowing to a halt."
        set_cmd_vel_linear 0.0
    fi
  fi 

  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
done


