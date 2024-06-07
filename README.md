## Usage
```bash
mkdir -p wbros_ws/src
cd wbros_ws/src
git clone git@github.com:realHarvey/Robotic-Vacuum.git
cd ..
catkin_make
```

## Include SLAM
`Cartographer`<br> 
`Hector`<br> 
`Gmapping`

## Env
`ROS Noetic`<br>
`Webots 2023b`

## Run in cartographer
if u wanna run Cartographer, you should copy `carto_cfg/cartographer.lua` to workspace of cartographer source, find a lua file named `revo_lds.lua` in `src/cartographer_ros/cartographer_ros/configuration_files`, replace it !

## Look
### room in webots
<img src="https://github.com/realHarvey/Robotic-Vacuum/assets/78713753/a5d2f28c-498f-4c47-acb4-3671df71808d" alt="maze" width="600"/>

### robot vacuum in webots
<img src="https://github.com/realHarvey/Robotic-Vacuum/assets/78713753/348cedac-3e93-45de-8574-f44e008e285b" alt="Roomba" width="300"/>

### rviz model
<img src="https://github.com/realHarvey/Robotic-Vacuum/assets/78713753/5d62199d-a8cc-4efd-8b92-f0cd8cc905c4" alt="rviz_screenshot_2024_05_31-01_31_09" width="300"/>

### successful tf tree
<img src="https://github.com/realHarvey/Robotic-Vacuum/assets/78713753/1f332b1c-0822-466c-894d-dc59a03e9d1a" alt="frames_all" width="600"/>

### successful ros graph
<img src="https://github.com/realHarvey/Robotic-Vacuum/assets/78713753/c0ef40ce-a0fe-430c-bab6-ae732c683966" alt="rosgraph" width="600"/>
