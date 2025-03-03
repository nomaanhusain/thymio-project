## Running FastDDS to send message from thymio to my pc

1. copy the setup-ros2-discovery.sh file to the thymio
2. In file change the ip to the ip of your pc (do ifconfig to get your ip), the line to change in the file would look like : export ROS_DISCOVERY_SERVER=134.34.231.109:11811
3. you should also have this file on your pc, here the ip should be localhost, i.e.: export ROS_DISCOVERY_SERVER=127.0.0.1:11811
4. source this setup file: ```source setup-ros2-discovery.sh```
4. now start the discovery server on your pc using ```fastdds discovery --server-id 0```
5. Now on your thymio source the setup file ```source setup-ros2-discovery.sh```. make sure here the ip is that of the machine running the server.
6. now run your publisher
7. you should now be able to see the topics on your pc. if not do a ```ros2 daemon stop``` and then list the topics again, everything should work

Note: In every terminal that you want to see the topic, you need to source the setup file. (on the pi with the ip of the PC, and on the pc with local host IP).

Note: Also to see the messages from the Vicon system on your pc, you need to colcon build and source the install of vicon_interfaces from einstein07, as it is custom messages it will give
type error otherwise.
