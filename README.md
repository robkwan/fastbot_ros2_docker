# fastbot_ros2_docker

1. For Task 1 Dockerize the simulation,

   inside the ROSject with the older docker version 1 installed.

   i. cd ~/ros2_ws/src/fastbot_ros2_docker/simulation

   ii. docker-compose up

   iii. docker ps

   iv. docker images


2. For Task 2 Dockerize the real robot,

   inside the Ubuntu OS of the fastbot, 
 
   a newer docker engine and the docker compose v2 are used.

   For installation,
   ```bash
   sudo apt-get install docker.io docker-compose 
   ```
   For Verification,
    ```bash
    docker --version
    docker-compose version
    ```
    
    Expected Outputs:
  
    Docker version 27.5.1, build ...

    docker-compose version 1.29.2, build unknown

   For running the docker,

   i. cd ~/ros2_ws/src/fastbot_ros2_docker

   ii. git clone https://github.com/robkwan/fastbot_ros2_docker.git
   
   iii. cd real/
   
   iv. docker-compose up

   v. docker ps

   In order to view the "ros2 topic list" correctly, it is found that the following settings may be needed:-

   - export ROS_IPV6=off
   - export ROS_DOMAIN_ID=1
   - unset CYCLONEDDS_URI
