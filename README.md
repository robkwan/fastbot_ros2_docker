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
   sudo apt-get install docker.io=27.5.1-0ubuntu3~22.04.2 docker-compose-plugin=2.39.1-1~ubuntu.22.04~jammy  


   It may be other than jammy if it is a different Ubuntu like focal, 

   For holding the versions to prevent upgrades,
   ```bash
   sudo apt-mark hold docker.io docker-compose-plugin

    For Verification,
    ```bash
    docker --version
    docker compose version

    Expected Outputs:
  
    Docker version 27.5.1, build ...

    Docker Compose version v2.39.1

   For running the docker,

   i. cd ~/ros2_ws/src/fastbot_ros2_docker

   ii. git clone https://github.com/robkwan/fastbot_ros2_docker.git
   
   iii. docker compose up

   iii. docker ps

   

