## How to get messages from vicon

1. First follow all the steps from the fastDDS guide

2. Go to the folder of the vicon receiver (https://github.com/einstein07/ros2-vicon-receiver/tree/humble)

3. Make sure ros2 is sourced, then source vicon_receiver/install/setup.bash

4. Go to the Vicon PC and do a ipconfig to ge the ip of the vicon pc

5. In the launch file here /home/nomaan/ros2-vicon-receiver/vicon_receiver/launch/client.launch.py replace the *ip* with the ip of the vicon pc

6. Go back to the ros2-vicon-receiver folder and launch the launch file: ros2 launch vicon_receiver client.launch.py

7. In another terminal (with ros2 sourced) if you do a ros2 topic list, you should see the topic for each object.

8. Now if you run the code on the thymio, everything should work

Note: This vicon-receiver was cloned from the humble branch using git clone -b humble https://github.com/einstein07/ros2-vicon-receiver.git

