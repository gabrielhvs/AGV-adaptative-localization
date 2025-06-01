<!-- Shields Section--><!-- Optional -->

<!-- 
* Insert project shields and badges through this link https://shields.io/
* 
*
-->

<div align="center">
    <a href="https://github.com/gabrielhvs/AGV-adaptative-localization/blob/main/LICENSE"><img alt="GitHub license" src="https://img.shields.io/github/license/gabrielhvs/AGV-adaptative-localization?color=ff69b4&style=for-the-badge"></a>
    <a href="https://github.com/gabrielhvs/AGV-adaptative-localization/stargazers"><img alt="GitHub stars" src="https://img.shields.io/github/stars/gabrielhvs/AGV-adaptative-localization?color=yellow&label=Project%20Stars&style=for-the-badge"></a>
    <a href="https://github.com/gabrielhvs/AGV-adaptative-localization/issues"><img alt="GitHub issues" src="https://img.shields.io/github/issues/gabrielhvs/AGV-adaptative-localization?color=brightgreen&label=issues&style=for-the-badge"></a>
    <a href="https://github.com/gabrielhvs/AGV-adaptative-localization/network"><img alt="GitHub forks" src="https://img.shields.io/github/forks/gabrielhvs/AGV-adaptative-localization?color=9cf&label=forks&style=for-the-badge"></a>
</div>
<br>


<!-- Logo Section  --><!-- Required -->

<!--
* Insert an image URL in the <img> "src" attribute bellow. (line )
* 
* Insert your github profile URL in the <a> "href" attribute bellow (line )
-->


<div align="center">
    <a href="https://github.com/gabrielhvs/AGV-adaptative-localization" target="_blank">
        <img src="image/agv_wallpaper.png?raw=true" 
        alt="Mambo Parrot" width="500" height="290">
    </a>
</div>


<!-- Project title 
* use a dynamic typing-SvG here https://readme-typing-svg.demolab.com/demo/
*
*  Instead you can type your project name after a # header
-->

<div align="center">
<img src="https://readme-typing-svg.demolab.com?font=Fira+Code&size=22&duration=4000&pause=5000&background=FFFFFF00&center=true&vCenter=true&multiline=true&width=900&lines=AGV-adaptative-localization">
</div>


## About
<!-- 
* information about the project 
* 
* keep it short and sweet
-->


This repository implements localization strategies for an AGV in an environment, with the aim of increasing accuracy using simulation methods such as Gazebo and communication structures with ROS. The main objective of this project is to increase knowledge about industrial AGV robots and current localization strategies.


## Requireds
<!-- 
* Here you may add information about how 
* 
* and why to use this project.-->

- machine with Ubuntu 20.04 or high
- clone this repository into your local machine.

```bash
    git clone https://github.com/gabrielhvs/AGV-adaptative-localization.git
```


## Demo<!-- Required -->
<!-- 
* You can add a demo here GH supports images/ GIFs/videos 
* 
* It's recommended to use GIFs as they are more dynamic
-->


<div align="center">
    <img alt="demo" src="./demo/amcl_demos.gif">
</div>

## Table of Contents<!-- Optional -->
<!-- 
* This section is optional, yet having a contents table 
* helps keeping your README readable and more professional.
* 
* If you are not familiar with HTML, no worries we all been there :) 
* Review learning resources to create anchor links. 
-->


<dev display="inline-table" vertical-align="middle">
<table align="center" vertical-align="middle">
        <tr>
            <td><a href="#about">About</a></td>        
            <td><a href="#demo">Demo</a></td>
            <td><a href="#documentation">Documentation</a></td>
        </tr>
        <tr>
            <td><a href="#contributors">Contributors</a></td>
            <td><a href="#feedback">Feedback</a></td>
            <td><a href="#contact">Contact</a></td>
        </tr>
</table>
</dev>


<p align="right"><a href="#about">back to top ⬆️</a></p>

## Documentation<!-- Optional -->
<!-- 
* You may add any documentation or Wikis here
-->

### Step 1 — Using Docker

It's need install docker in your desktop follow this tutorial: [docker install](docker-base/README.md)

To using a docker image of this repository run this comand in terminal:

```
./build.sh [folder of Dockerfile]
```

For this repository:
```
./build.sh docker-base
```

After this to using the image created run this:
```
./open.sh [folder of Dockerfile] [work dir]
```

For this repository:
```
./open.sh docker-base ws
```

When execute this command, a iteractive terminal is open. If you needs close this using:
```
./stop.sh [work dir]
```

For this repository:
```
./stop.sh ws
```

### Step 2 — Building Ros Packages

Using the conteiner docker run this command to start building of packages ros:
```
catkin build
```

Source the link ros in work dir:
```
source devel/setup.sh
```

### Step 3 — Run Ros Packages

To start AGV in a empty word run this package:
```
roslaunch agv_simulation agv_gazebo_emptyworld.launch
```

#### Mapping
To mapping using this repository, you can start using this:
```
roslaunch adaptative_localization AGV_Mapping.launch
```
This launch file starting gmaping package and insert AGV in Gazebo environment.

Open teleoperation in other terminal in docker to, using this command:
```
rosrun teleop_twist_keybord teleop_twist_keybord.py
```
Control robot in the world and mapping this environment.
When finish the mapping in environment, to save a map using this comand:

```
rosrun map_server map_saver -f ~ws/src/adaptative_localization/maps/mymap
```
#### Localization
To localization AGV using this map you can run this command:
```
roslaunch adaptative_localization AGV_Amcl.launch
```
This launch file open gazebo and rviz of robot with particles points that represent a localizaion.
Open teleop_twist_keybord to navigation in the world, the particle points should follow the robot.
Else particles doesn't follow the robot change the global frame to odom.

#### Path trajetory

To path trajetory teste it's possible using this command:
```
roslaunch adaptative_localization AGV_moving.launch
```
This launch file started gazebo and rviz, in the rviz you can apply a goal and
the package  generate a path until a goal. The robot will be control to follow a path.

#### Sensor Fusion (EKF)

To test sensor fusion using the `robot_localization` package, you can run the following launch file:

```
roslaunch adaptative_localization ekf_localization.launch
```
This launch file starts:

The AGV robot in the standard simulation environment (map_depo.world)

The EKF filter from the robot_localization package

RViz with a configuration for real-time observation

Publication of filtered odometry to the topic /odometry/filtered

Notes:
The reference frame used is base_link.

The EKF configuration file is located at:
ws/src/adaptative_localization/config/ekf.yaml

#### More detais about packages

Package | Documentation
--------|--------------
agv     | [README.md](ws/src/agv/README.md)
ros_autonomous_slam | [README.md](ws/src/mapping/ros_autonomous_slam/README.md)
slam_gmapping       | [README.md](ws/src/mapping/slam_gmapping/README.md)
teleop_twist_keyboard |[README.md](ws/src/operation/teleop_twist_keyboard/README.md)
navigation          | [README.md](ws/src/particle_filter/navigation/README.md)
robot_line_follower | [README.md](ws/src/robot_line_follower/README.md)
robot_localization |[README.md](ws/src/sensor_fusion/robot_localization/README.md)

[Main package](ws/src/adaptative_localization)

## Contributors<!-- Required -->
<!-- 
* Without contribution we wouldn't have open source. 
* 
* Generate github contributors Image here https://contrib.rocks/preview?repo=angular%2Fangular-ja
-->

<a href="https://github.com/gabrielhvs/AGV-adaptative-localization/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=gabrielhvs/AGV-adaptative-localization" />
</a>

## Feedback<!-- Required -->
<!-- 
* You can add contacts information like your email and social media account 
* 
* Also it's common to add some PR guidance.
-->


> You can make this project better, please  feel free to open a [Pull Request](https://github.com/gabrielhvs/AGV-adaptative-localization/pulls).
- If you notice a bug or a typo use the tag **"Correction"**.
- If you want to share any ideas to help make this project better, use the tag **"Enhancement"**.

<details>
    <summary>Contact Me 📨</summary>

### Contact<!-- Required -->
Reach me via email: [gabbrielvasc@gmail.com](mailto:gabbrielvasc@gmail.com)
<!-- 
* add your email and contact info here
* 
* 
-->
    
</details>

<!-- - Use this html element to create a back to top button. -->
<p align="right"><a href="#about">back to top ⬆️</a></p>
