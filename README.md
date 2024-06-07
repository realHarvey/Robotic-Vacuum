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