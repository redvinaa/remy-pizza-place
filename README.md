# REMY Robotics Test Task
Test task by Remy Robotics: implement create an automatized pizza factory. The skeleton of the project is implemented in this repo.

## Usage
Build and run the container with `docker-compose up --build`. This launches all the processes needed, including RViz and Gazebo.
For these to be seen, the X server is forwarded.

After that, enter the container with `docker exec -it remy_pizza_place bash`, source the ros entrypoint script with
`. /ros_entrypoint.sh` and call the following service to start the pick and place process: `rosservice call /stage1/task1`.
In the simulation windows the robot should start moving.
