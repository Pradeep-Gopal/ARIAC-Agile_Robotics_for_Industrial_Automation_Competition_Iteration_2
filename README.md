# ARIAC Competition

Picking and delivering parts in an industrial environment with faulty parts

## Results
[Scenario 4](https://www.youtube.com/watch?v=Xu1gFQL5WeM&list=PL_HqcgW4roXofxZxdxJUef4rXWphtUOpH&index=1)

To view previous scenarios and results, please click the link below

[Prev Results](https://github.com/Pradeep-Gopal/ARIAC---Agile-Software-Development-for-Robots)

## Team members
1. Pradeep Gopal
2. Rajesh 
3. Govind
4. Dakota Abernathy
5. Cheng Chen

## Steps to Run the package

Install Ariac package in your workspace using the steps mentioned in the following link

[Ariac Installation Instructions](https://github.com/usnistgov/ARIAC/blob/master/wiki/tutorials/installation.md)


Follow these instructions to run the package after installing ARIAC

1. git clone the package in the /ariac_ws/src/ directory and rename the package to rwa4_group1
2. Open a terminal and type the following commands
3. cd /ariac_ws
4. catkin build rwa4_group1
5. source devel/setup.bash
6. roslaunch rwa4_group1 rwa4.launch load_moveit:=true

Wait till the terminal says "you can start planning now"

7. Open a new terminal and enter the following command to run the node.
8. cd /ariac_ws
9. source devel/setup.bash
10. rosrun rwa4_group1 rwa4_node

