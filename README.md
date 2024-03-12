# 3 Steps to Deploy Autoware.universe with Using Carla_0.9.15 on Jetson Orin

This repository is based on the github repository from TUM. [**LINK**](https://github.com/TUMFTM/Carla-Autoware-Bridge.git) 

Thanks a lot for their brilliant work, if you are interested in their paper, you can refer to this link [[PDF](https://arxiv.org/abs/2402.11239)]

In this repository, we did some revisions to use **REV-shuttle-bus** in the **Carla simulator**, which has different sensors, also the **autoware.universe** version we used will be changed to our version later.

## **Requirements**

### Software
1. Carla 0.9.15 (the newest version)
2. Autoware.universe (V1.0 Branch, will be changed to other version)
3. Autoware-Carla-Bridge (Main Branch)
4. ROS2 (Humble version)
5. Ubuntu 22.04 (LTS Released version)

### Hardware

Jetson Orin Developer kit is an awesome platform for robots and Autonomous Vehicle.

1. One all 1 Gigabyte Switcher or Router (for communication between your host Ubuntu machine with Jetson Orin).

2. One Jetson Orin Develop kit 32GB/64GB with nvme ssd 500G external storage.
3. One Ubuntu 22.04 Host Machine.
4. Two or more Cat 5e/6 or above level cables 

All devices are required to connect with each other in a local network. All 1 Gigabyte specs are better. 

------

***Note:*** To illustrate the following steps easier, I use "***Host Machine***" stands for the machine which has a standard **GPU** device like **RTX2070** or 30/40 series and it 's also the one which will deploy a **Carla 0.9.15** simulator and **Autoware-Carla-Bridge ROS2** packages, and use "***Jetson Orin***" stands for the embedded system which will be used for deploying the **Autoware.universe** stack.

------

> Before you go through the following steps, I suggest you pull some necessary docker images using following commands, so after you finish reading this document, you can start deploying right away:

```bash
 $ docker pull carlasim/carla:0.9.15
 $ docker pull tumgeka/carla-autoware-bridge:latest
 $ docker pull 1429053840/autoware.universe-carla-0.9.15:humble-20240215-cuda-arm64-v0.1
```



## 3 Steps to deploy

### STEP-1: Deploy the carla-0.9.15 simulator

This step is based on the carla official docker image, you have to pull the image and then run it on your Host machine.

You have to check the rpc-port is not used on your Host machine.

```bash
$ docker pull carlasim/carla:0.9.15
$ docker run --privileged --gpus all --net=host -e DISPLAY=$DISPLAY carlasim/carla:0.9.15 /bin/bash ./CarlaUE4.sh -carla-rpc-port=1403
```

### STEP-2: Deploy the autoware-carla-bridge

This step is also based on the docker image issued by TUM team. Setting the timeout to a larger volume is necessary on my own laptop, the default value is 5000. 

```bash
$ docker pull tumgeka/carla-autoware-bridge:latest
$ docker run -it -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp --network host tumgeka/carla-autoware-bridge:latest
$ ros2 launch carla_autoware_bridge carla_aw_bridge.launch.py port:=1403 town:=Town10HD timeout:=10000
```

### STEP-3: Deploy autoware.universe stack on Jetson Orin

Compile the autoware.universe is sometimes difficult, as there are various dependencies. So, I suggest that use our pre-built docker images is easier. These following commands need to run on the Jetson Orin terminals. As we want to use a Jetson Orin device to deploy the autoware.universe stack. 

***Note**:* *This docker image is a little bit large and only supported on the ARM64 platform, so Jetson series are all suitable.*

```bash
$ docker pull 1429053840/autoware.universe-carla-0.9.15:humble-20240215-cuda-arm64-v0.1
$ sudo apt-get install python3-rocker
$ rocker --user --nvidia --privileged --network host --x11 --volume $HOME/Documents  --volume $HOME/autoware -- 1429053840/autoware.universe-carla-0.9.15:humble-20240215-cuda-arm64-v0.1
$ ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=carla_t2_vehicle sensor_model:=carla_t2_sensor_kit map_path:=/autoware1.0_ws/Town10/
```

***Note:*** When you run the rocker commands, and it may shows the error like the following messages:

. You merely need to change the "--gpus all" to "--runtime=nvidia". like the example commands below: "Your commands will be different with mine, you should copy the commands, after you run the ***rocker --user --nvidia xxxx** command*"

```bash
$ docker run  --rm -it --network host   --runtime=nvidia --privileged  -e DISPLAY -e TERM   -e QT_X11_NO_MITSHM=1   -e XAUTHORITY=/tmp/.dockerafc7hfmf.xauth -v /tmp/.dockerafc7hfmf.xauth:/tmp/.dockerafc7hfmf.xauth   -v /tmp/.X11-unix:/tmp/.X11-unix   -v /etc/localtime:/etc/localtime:ro  8ea8cd5cadfe
```



### Optional

#### Spawn NPCs

If you want to add some NPCs in the carla you can use the python script in the autoware-carla-brdige docker container. Must set the port to the same with the front steps. This ***generate_traffic.py*** should be used in the docker container's terminal of the STEP-2(tumgeka/carla-autoware-bridge:latest).

```bash
$ python3 src/carla_autoware_bridge/utils/generate_traffic.py -p 1403
```

#### Deploy autoware.universe stack on Host Machine

Compile the autoware source codes on your host machine locally, if you do not want to deploy on the jetson embedded systems.

```bash
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If you want to compile the source codes on the Host Machine with a GPU card, you may need to solve some dependencies problems.







## Local workflow

Local workflow will be added later, add a Dockerfile, released and revised source codes. 



TODO List

- [ ] Add another shuttle-bus model in carla simulator to use
- [ ] Simplified a Dockerfile for users to build the Autoware.universe docker image locally.
- [ ] Uploads the revised autoware.universe codes for solving the problems of starting the perception module in the autoware.universe stack











