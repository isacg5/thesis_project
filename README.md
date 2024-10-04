# Master Thesis Project

This project presents a system developed in ROS2, utilizing PlanSys2 to create a plan for performing search and rescue tasks using the Spot robot from Boston Dynamics. The repository contains all files related to the project.

The entire system has been developed within a Docker environment. To simplify installation and configuration, you can access the full setup with all dependencies included by clicking [here](https://hub.docker.com/repository/docker/isacg5/final_solution/general).

## How to run?
In order to connect with the ZED2 camera, the server is activated with the
following command inside the Docker container, but be aware that the client must be running
from the camera side:
- cd /ros2_ws/src/zed/scripts
- python3 server_both.py

In order to execute the system, the steps to follow are the following:
1. cd /ros2_ws/src/plansys/plansys2_simple_example_py/plansys2_simple_example_py
2. python3 gotopoint.py
3. ros2 launch isabel_interface all.launch.py (check that the command +xhost was previosly wrote in the terminal outside the Docker container in order to visualize RViZ)
4. ros2 launch plansys2_simple_example_py plansys2_launch.py 
5. ros2 run plansys2_simple_example_py controller_node 

## Thesis report
You can download the thesis project [here](https://github.com/isacg5/thesis_project/blob/main/thesis/s5715592_Cebollada_Gracia_Isabel.pdf)


## Experiments
The system has been thoroughly tested through simulated actions and a real-world scenario. The results of these experiments are presented in the following videos:
<p align="justify">
You can watch the first experiment here
</p>

[![Watch the video](https://github.com/isacg5/thesis_project/blob/main/resources/1exp.png)](https://youtu.be/avZONHfUlhs)

<p align="justify">
You can watch the second experiment here
</p>

[![Watch the video](https://github.com/isacg5/thesis_project/blob/main/resources/2exp.png)](https://youtu.be/G9KxLVNw7Q8)

<p align="justify">
You can watch the third experiment here
</p>

[![Watch the video](https://github.com/isacg5/thesis_project/blob/main/resources/3exp.png)](https://youtu.be/qOSk-DQj6ss)
