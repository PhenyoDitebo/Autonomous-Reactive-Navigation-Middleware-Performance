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

# --------------------------------- GLOBAL VARIABLES ----------------------------------

threshold=0.5 # I experimented with ranges, and 0.5 is the smallest we can go without crash issues

# Turn Distance Logic
# I want to be able to check all 45 degree increments of the front
# hence, I need the the robot to spin at exactly 45 degrees when it seems something in front
# we can use simple math for this to avoid having to access any more ray_arrays than we need to.
# However, we are in rad, so these numbers are in rad or rad/s
# turn_dist = pi/4 = 0.785398 rad = turn_speed * turn_time
# we need to break these two numbers down.
# turn_speed needs to be slow to avoid scan skewing, hence 

turn_speed=0.5 #rad/sec
turn_time=$(echo "0.785398 / $turn_speed" | bc -l) # sec from turn_dist/turn_speed = turn_time

# speeds of linear movement
forw_speed=0.05 # m/s
back_speed=-0.05 # m/s

buffer=0.1 # meters (or 10cm) (to avoid collisions due to momentum issues)

# main while loop for naive obstacle avoider
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"

while :
do
# --------------------------------- PRIORITY TREE -----------------------------------

  # Priority tree BASE (trunk); Priority 1: FRONT
  # 1. GET THE FRONT RANGE
  ff_range=$(get_scan_front_ray_range)
  
  # if the sensor sees we have a large distance ahead, or detects nothing (computer failure)
  # assign it a large number we can use for calculation of the decision tree later.
  if [[ "$ff_range" == "inf" || -z "$ff_range" ]]; then ff_range=10.0; fi

  # 2. CHECK IF FRONT IS FREE using simple bool logic
  # returns a boolean for 0 (obstacle ahead) or 1 (Nothing ahead)
  front_free=$(echo "$ff_range > $threshold" | bc -l)

  if [[ "$front_free" == "0" ]]; then
    echo "Obstacle Detected! Breaking."
    set_cmd_vel_linear 0.000
    set_cmd_vel_angular 0.000

    # CASE 1: FORWARD_FRONT path blocked -> decide direction

    # --- GATHER ALL FRONT PATH DIRECTIONS ---  
    fl_range=$(get_scan_front_left_ray_range)
    fr_range=$(get_scan_front_right_ray_range)
    ll_range=$(get_scan_left_ray_range)
    rr_range=$(get_scan_right_ray_range)
    bb_range=$(get_scan_back_ray_range)

    # Clean up the side ranges to avoid crashes within the logic
    # since the math cannot work with 'inf' and we wish to avoid crashes if
    # we have nothing. 
    if [[ "$fl_range" == "inf" || -z "$fl_range" ]]; then fl_range=10.0; fi
    if [[ "$fr_range" == "inf" || -z "$fr_range" ]]; then fr_range=10.0; fi
    if [[ "$ll_range" == "inf" || -z "$ll_range" ]]; then ll_range=10.0; fi
    if [[ "$rr_range" == "inf" || -z "$rr_range" ]]; then rr_range=10.0; fi
    if [[ "$bb_range" == "inf" || -z "$bb_range" ]]; then bb_range=10.0; fi

    # Evaluate if we have any paths to move forward.


    # ---------------------------- PRIORITY BRANCHES -------------------------------
    # PRIORITY 1.2: FRONT_LEFT
    # PRIORITY 1.3: FRONT_RIGHT
    # PRIORITY 2: LEFT
    # PRIORITY 3: RIGHT
    # PRIORITY 4: BACK

    # Priority 1.2: if front_left is clear AND greater than front_right, turn slight left
    if (( $(echo "$fl_range > $threshold && $fl_range > $fr_range" | bc -l) )); then
        echo "DECISION: Turning Front_Left. More space found here than FR (P1.2)."
        set_cmd_vel_linear 0.000
        set_cmd_vel_angular $turn_speed
        sleep $(echo "0.5 * $turn_time" | bc -l)

    # Priority 1.3: if front_right is clear AND greater than front_left, turn slight right
    elif (( $(echo "$fr_range > $threshold && $fr_range > $fl_range" | bc -l) )); then
        echo "DECISION: Turning Front_Right. More space found here than FL (P1.3)."
        set_cmd_vel_linear 0.000
        set_cmd_vel_angular $turn_speed
        sleep $(echo "0.5 * $turn_time" | bc -l)

    # Priority 2: left
    elif (( $(echo "$ll_range > $threshold && $ll_range > $rr_range" | bc -l) )); then
        echo "DECISION: Turning LEFT. More space found here than RIGHT (P2)."
        set_cmd_vel_linear 0.000
        set_cmd_vel_angular $turn_speed
        sleep $turn_time

    # Priority 3: Turn right
    elif (( $(echo "$rr_range > $threshold && $rr_range > $ll_range" | bc -l) )); then
        echo "DECISION: Turning RIGHT. More space found here than LEFT (P3)."
        set_cmd_vel_linear 0.000
        set_cmd_vel_angular $turn_speed
        sleep $turn_time

    else 
        # Priority 4: Move backwards.

        # Check if we are genuinely boxed in on both sides.
        left_tight=$(echo "$ll_range <= $threshold" | bc -l)
        right_tight=$(echo "$rr_range <= $threshold" | bc -l)
        back_blocked=$(echo "$bb_range <= $threshold" | bc -l)

        if [[ "$left_tight" == "0" && "$right_tight" == "0" ]]; then
                if [[ "$back_blocked" == "0" ]]; then
                    echo "DECISION: Trapped on sides & Front. Rear clear. Reversing..."
                    set_cmd_vel_angular 0.000
                    set_cmd_vel_linear $back_speed
                    sleep 0.5
                    set_cmd_vel_linear 0.000

                else
                    echo "CRITICAL DECISION: All 4 axes blocked! Emergency Stop."
                    set_cmd_vel_linear 0.000
                    set_cmd_vel_angular 0.000
                fi
            else 
                # Symmetrical tie-breaker (this just keeps happening like a fan): 
                # The ranges are equal but there's open room.
                # Force a default left turn to break the logic deadlock.
                echo "DECISION: Symmetry detected with open space. Forcing Left pivot."
                set_cmd_vel_linear 0.000
                set_cmd_vel_angular $turn_speed
                sleep $(echo "1 * $turn_time" | bc -l)
            fi
        fi

    set_cmd_vel_angular 0.000
    echo "Clear Path Found, stopping rotation."
    sleep 1.000 # gives a lot of time for robot to settle

  else 
    # Case 2: path clear -> move forward
    dist_to_move=$(echo "$ff_range - $threshold" | bc -l) # to avoid collision

    if (( $(echo "$dist_to_move > $buffer" | bc -l) )); then
    set_cmd_vel_linear $forw_speed # reduced speed to avoid momentum-caused crashes
    sleep 0.05 # check sensors every 50ms not 100ms
    fi
  fi 

  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
done