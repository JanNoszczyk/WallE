source /opt/ros/foxy/setup.bash

# Go to root folder, parent folder of src

rosdep install -i --from-path src --rosdistro foxy -y #This is optional, checks for missing dependencies

colcon build --packages-select walle_driver

. install/setup.bash

ros2 run walle_driver talker

# Now in a new terminal window, source ros2 and install/setup.bash again
ros2 run walle_driver listener