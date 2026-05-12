#! /usr/bin/bash

# include the functions library
source ./robot_functions.bash

# robot statistics

# this is an infinite while loop - use ctrl+c to break
echo "Running Robot Statistics with Bash Script..."
echo "Press Ctrl+C to Terminate..."

# main while loop for naive obstacle avoider
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
while :
do
  # print distance covered since start
  echo "Distance Covered: $(get_odom_distance)m." 

  # print current direction of robot
  echo "Direction: $(get_odom_direction)."

  # print odom position x, y, z
  echo "Position x: $(get_odom_position_x)m."
  echo "Position y: $(get_odom_position_y)m."
  echo "Position z: $(get_odom_position_z)m."

  # print odom orientation r, p, y
  echo "Orientation Roll: $(get_odom_orientation_r)rad."
  echo "Orientation Pitch: $(get_odom_orientation_p)rad."
  echo "Orientation Yaw: $(get_odom_orientation_y)rad."

  # print imu angular velocity x, y, z
  

  # print imu linear acceleration x, y, z
  # ...

  # print a divider line to show iteration is complete
  echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
done

# End of Code

