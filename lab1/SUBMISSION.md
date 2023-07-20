# Lab 0: Docker and ROS 2

## Docker Hub ID
(FILL ME IN)

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer: Executing the first command will make ROS2's packaes available for use in the current terminal. While executing the second command enables the user to add new packages without interfering with the existin ROS 2 workspace that the user is extending. If the two commands are executed in order, the first command would be the underlay while the second one will be the overlay.

### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer: The queue size argument limits the amount of queued messages if a subscriber is not receiving them fast enough. With a bigger queue size messages are less likely to be dropped. 

### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer: For the first case it is not necessary to execute colcon build while for the second case it is necessary.

### Q4 (optional): While completing this lab, are there any parts of the tutorial and lab instruction that could be improved?

Anwer: No, the lab provides links for help. The only improvement I can suggest is to give specific links eg. tutorials in the ros2 documentation that is directly related to the lab.
