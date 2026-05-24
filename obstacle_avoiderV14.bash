#!/usr/bin/bash

# include the functions library
source ./robot_functions.bash

# --------------------------------- GLOBAL VARIABLES ----------------------------------

threshold=0.3
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

buffer=0.17 # (to avoid collisions due to momentum issues)

# The absolute closest the robot can ever get to a wall
safety_margin=$(echo "$threshold + $buffer" | bc -l)
current_state="stopped"

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


# -------------------------------------- FUNCTIONS --------------------------------------
move_forward() {
    if [[ "$current_state" != "forward" ]]; then
        set_cmd_vel_angular 0.000
        set_cmd_vel_linear $forw_speed
        current_state="forward"
    fi
}

move_backward() {
    if [[ "$current_state" != "backward" ]]; then
        set_cmd_vel_angular 0.000
        set_cmd_vel_linear $back_speed
        current_state="backward"
    fi
}

stop_moving() {
# stop moving
    set_cmd_vel_linear 0.000
    set_cmd_vel_angular 0.000
    current_state="stopped"
}

rotate_right() {
    # rotate to the right at an angle of 45 degrees.
    local multiplier=${1:-1}

    set_cmd_vel_linear 0.000
    set_cmd_vel_angular -$turn_speed
    current_state="turning" # Clear the forward lock

    local total_sleep=$(echo "$turn_time * $multiplier" | bc -l)
    sleep $total_sleep
    set_cmd_vel_angular 0.00
    
    # time to populate fresh data before the main loop reads it
    sleep 0.1 
}

rotate_left() {
    # rotate to the left at an angle of 45 degrees.
    local multiplier=${1:-1}

    set_cmd_vel_linear 0.000
    set_cmd_vel_angular $turn_speed
    current_state="turning" # Clear the forward lock
    
    local total_sleep=$(echo "$turn_time * $multiplier" | bc -l)
    sleep $total_sleep
    set_cmd_vel_angular 0.00
     
    # time to populate fresh data before the main loop read it
    sleep 0.1
}

# --------------------------------- MAIN FUNCTIONS --------------------------------
search_for_path() {
    # search 
    # CASE 1: FORWARD_FRONT path blocked -> decide direction

    # --- GATHER ALL FRONT PATH DIRECTIONS ---  
    fl_range=$(get_scan_front_left_ray_range)
    fr_range=$(get_scan_front_right_ray_range)
    ll_range=$(get_scan_left_ray_range)
    rr_range=$(get_scan_right_ray_range)
    bb_range=$(get_scan_back_ray_range)

    # Clean up the side ranges to avoid crashes within the logic
    # since the math cannot work with 'inf' or no data 
    # inf - space, so move
    # no data? - stop. could be a bug
    # if [[ "$fl_range" == "inf" ]]; then fl_range=10.0; fi
    if [[ -z "$fl_range" ]]; then fl_range=0.0; fi
    
    # if [[ "$fr_range" == "inf" ]]; then fr_range=10.0; fi
    if [[ -z "$fr_range" ]]; then fr_range=0.0; fi
    
    # if [[ "$ll_range" == "inf" ]]; then ll_range=10.0; fi
    if [[ -z "$ll_range" ]]; then ll_range=0.0; fi
    
    # if [[ "$rr_range" == "inf" ]]; then rr_range=10.0; fi
    if [[ -z "$rr_range" ]]; then rr_range=0.0; fi
    
    # if [[ "$bb_range" == "inf" ]]; then bb_range=10.0; fi
    if [[ -z "$bb_range" ]]; then bb_range=0.0; fi

    # Evaluate if we have any paths to move forward.

    # ---------------------------- PRIORITY BRANCHES -------------------------------
    # PRIORITY 1.2: FRONT_LEFT
    # PRIORITY 1.3: FRONT_RIGHT
    # PRIORITY 2: LEFT
    # PRIORITY 3: RIGHT
    # PRIORITY 4: BACK

    # Priority 1.2: if front_left is clear AND greater than front_right, turn slight left
    if [[ "$(echo "$fl_range > $threshold && $fl_range > $fr_range" | bc -l)" -eq 1 ]]; then
        echo "DECISION: Turning Front_Left. More space found here than FR (P1.2)."
        rotate_left 0.5

    # Priority 1.3: if front_right is clear AND greater than front_left, turn slight right
    elif [[ "$(echo "$fr_range > $threshold && $fr_range > $fl_range" | bc -l)" -eq 1 ]]; then
        echo "DECISION: Turning Front_Right. More space found here than FL (P1.3)."
        rotate_right 0.5

    # Priority 2: left
    elif [[ "$(echo "$ll_range > $threshold && $ll_range > $rr_range" | bc -l)" -eq 1 ]]; then
        echo "DECISION: Turning LEFT. More space found here than RIGHT (P2)."
        rotate_left 1

    # Priority 3: Turn right
    elif [[ "$(echo "$rr_range > $threshold && $rr_range > $ll_range" | bc -l)" -eq 1 ]]; then
        echo "DECISION: Turning RIGHT. More space found here than LEFT (P3)."
        rotate_right 1

    else 
        # Priority 4: Move backwards.

        # Check if we are genuinely boxed in on both sides.
        left_clear=$(echo "$ll_range > $threshold" | bc -l)
        right_clear=$(echo "$rr_range > $threshold" | bc -l)
        back_clear=$(echo "$bb_range > $threshold" | bc -l)

        if [[ "$left_clear" == "0" && "$right_clear" == "0" && "$back_clear" == "1" ]]
        then
            echo "DECISION: Trapped on sides & Front. Rear clear. Reversing..."
            move_backward

        elif [[ "$left_clear" == "0" && "$right_clear" == "0" && "$back_clear" == "0" ]]
        then
            echo "CRITICAL DECISION: All 4 axes blocked! Emergency Stop."
            stop_moving

        else 
            # Symmetrical tie-breaker (this just keeps happening like a fan): 
            # The ranges are equal but there's open room.
            # Force a default left turn to break the logic deadlock.
            echo "DECISION: Symmetry detected with open space. Forcing Left pivot."
            rotate_left 1
        fi
    fi

    set_cmd_vel_angular 0.000
    echo "Clear Path Found, stopping rotation."
    sleep 0.05
}

run_nav_cycle() {
    # Priority tree BASE (trunk); Priority 1: FRONT
    # 1. GET THE FRONT RANGE
    ff_range=$(get_scan_front_ray_range)
    echo "Front sensor reading: $ff_range"
    
    # if the sensor sees we have a large distance ahead, or detects nothing (computer failure)
    # assign it a number we can use for calculation of the decision tree later.
    # if [[ "$ff_range" == "inf" ]]; then ff_range=10.0; fi
    if [[ -z "$ff_range" ]]; then ff_range=0.0; fi

    # 2. CHECK IF FRONT IS FREE using simple bool logic
    # returns a boolean for 0 (obstacle ahead) or 1 (Nothing ahead)
    front_clear=$(echo "$ff_range > $safety_margin" | bc -l)

    # CASE 1: FORWARD_FRONT path blocked -> decide direction
    if [[ "$front_clear" == "0" ]]; then
        echo "Obstacle Detected! Breaking."
        stop_moving
        search_for_path

    else 
        # Case 2: path clear -> move forward
        move_forward
    fi 
}

main() {
    # runs everything keeping information under the hood.
    echo "Initililizing system..."
    set_cmd_vel_linear 0.000
    set_cmd_vel_angular 0.000
    echo "System Active."

    while :
    do
        run_nav_cycle
    done
}

# start program
main
