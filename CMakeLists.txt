cmake_minimum_required(VERSION 2.8.3)
project(webots_ros)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## 查找catkin宏和库
find_package(catkin REQUIRED COMPONENTS 
        roscpp rospy 
        std_msgs sensor_msgs 
        message_generation 
        tf
        moveit_ros_planning_interface
)
## 通过 CMake 的约定找到系统依赖项
# find_package(Boost REQUIRED COMPONENTS system)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -save-temps=obj")
add_compile_options(-pg)
#######################################
## Declare ROS messages and services ##
#######################################

## 在 msg 文件夹中生成消息
add_message_files(
    FILES
    BoolStamped.msg
    Float64Stamped.msg
    Int32Stamped.msg
    Int8Stamped.msg
    RadarTarget.msg
    RecognitionObject.msg
    RecognitionObjects.msg
    StringStamped.msg
    ContactPoint.msg
)

## 在 srv 文件夹中生成服务
add_service_files(
    FILES
    camera_get_focus_info.srv
    camera_get_info.srv
    camera_get_zoom_info.srv
    display_draw_line.srv
    display_draw_oval.srv
    display_draw_pixel.srv
    display_draw_polygon.srv
    display_draw_rectangle.srv
    display_draw_text.srv
    display_get_info.srv
    display_image_copy.srv
    display_image_delete.srv
    display_image_load.srv
    display_image_new.srv
    display_image_paste.srv
    display_image_save.srv
    display_set_font.srv
    field_disable_sf_tracking.srv
    field_enable_sf_tracking.srv
    field_get_bool.srv
    field_get_color.srv
    field_get_count.srv
    field_get_float.srv
    field_get_int32.srv
    field_get_node.srv
    field_get_rotation.srv
    field_get_string.srv
    field_get_type.srv
    field_get_name.srv
    field_get_vec2f.srv
    field_get_vec3f.srv
    field_import_node_from_string.srv
    field_remove_node.srv
    field_remove.srv
    field_set_bool.srv
    field_set_color.srv
    field_set_float.srv
    field_set_int32.srv
    field_set_rotation.srv
    field_set_string.srv
    field_set_vec2f.srv
    field_set_vec3f.srv
    get_bool.srv
    get_float_array.srv
    get_float.srv
    get_int.srv
    get_string.srv
    get_uint64.srv
    get_urdf.srv
    gps_decimal_degrees_to_degrees_minutes_seconds.srv
    lidar_get_frequency_info.srv
    lidar_get_info.srv
    lidar_get_layer_point_cloud.srv
    lidar_get_layer_range_image.srv
    motor_set_control_pid.srv
    mouse_get_state.srv
    node_add_force_or_torque.srv
    node_add_force_with_offset.srv
    node_disable_contact_points_tracking.srv
    node_disable_pose_tracking.srv
    node_enable_contact_points_tracking.srv
    node_enable_pose_tracking.srv
    node_get_center_of_mass.srv
    node_get_contact_points.srv
    node_get_field.srv
    node_get_field_by_index.srv
    node_get_id.srv
    node_get_number_of_fields.srv
    node_get_name.srv
    node_get_orientation.srv
    node_get_pose.srv
    node_get_parent_node.srv
    node_get_position.srv
    node_get_static_balance.srv
    node_get_status.srv
    node_get_string.srv
    node_set_string.srv
    node_get_type.srv
    node_get_velocity.srv
    node_remove.srv
    node_reset_functions.srv
    node_move_viewpoint.srv
    node_set_joint_position.srv
    node_set_visibility.srv
    node_set_velocity.srv
    pen_set_ink_color.srv
    range_finder_get_info.srv
    receiver_get_emitter_direction.srv
    robot_get_device_list.srv
    robot_set_mode.srv
    robot_wait_for_user_input_event.srv
    save_image.srv
    set_bool.srv
    set_float.srv
    set_float_array.srv
    set_int.srv
    set_string.srv
    skin_get_bone_name.srv
    skin_get_bone_orientation.srv
    skin_get_bone_position.srv
    skin_set_bone_orientation.srv
    skin_set_bone_position.srv
    speaker_is_sound_playing.srv
    speaker_speak.srv
    speaker_play_sound.srv
    supervisor_get_from_def.srv
    supervisor_get_from_id.srv
    supervisor_get_from_string.srv
    supervisor_movie_start_recording.srv
    supervisor_set_label.srv
    supervisor_virtual_reality_headset_get_orientation.srv
    supervisor_virtual_reality_headset_get_position.srv
)

## 使用此处列出的任何依赖项生成添加的消息和服务
generate_messages(
    DEPENDENCIES
    std_msgs
    sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS 
    roscpp 
    std_msgs sensor_msgs 
    message_runtime
)

###########
## Build ## 
###########
include_directories(
    ${catkin_INCLUDE_DIRS}
    ~/wbros_ws/src/webots_ros/src/include
)

add_executable(ccontroller src/controller.cc)
add_dependencies(ccontroller webots_ros_generate_messages_cpp)
target_link_libraries(ccontroller ${catkin_LIBRARIES})

install(TARGETS ccontroller
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


#############
## Install ## 
#############
catkin_install_python(PROGRAMS scripts/ros_controller.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/ros_python.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
catkin_install_python(PROGRAMS scripts/webots_launcher.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY plugins DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY worlds DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY controllers DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
