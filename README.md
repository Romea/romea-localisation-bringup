# 1 Overview #

This package provides a robot to world localisation node


# 2 Node #

### 2.1 Subscribed Topics ###

- localisation/twist (romea_odo_msgs::KinematicMeasureStamped)

- localisation/angular_speed ( romea_localisation_msgs::ObservationAngularSpeedStamped)

- localisation/course (romea_localisation_msgs::ObservationCourseStamped)

- localisation/position (romea_localisation_msgs::ObservationPositionStamped)

- localisation/range (romea_localisation_msgs::ObservationRangeStamped)
        
- joy(sensor_msgs::Joy)


### 2.2 Published Topics ###

- filtered_odom (nav_msgs::Odometry)

### 2.3 Parameters ###


- ~controller (string, default: fsm) 

    The node can be controlled in using joystick or romea finite state machine (fsm).   
    By default the node is controlled by finite state machine. When joystick is used  
    four buttons are availables :  
    &nbsp;&nbsp;&nbsp;&nbsp;- A : start localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;- B : reset localisation  

- ~angular_speed_source (string, default odometry)

    Name of angular source speed :
    &nbsp;&nbsp;&nbsp;&nbsp;- angular_speed : data is read from localisation/angular_speed topic
    &nbsp;&nbsp;&nbsp;&nbsp;- twist : data is read from localisation/twist topic

- ~minimal_linear_speed_rate(float, default: 10) 

    Miminal rate of linear speed data. Linear speed can come from odometry  
    or joint_state messages 

- ~minimal_angular_speed_rate(float, default: 10) 

    Miminal rate of angular speed data. Angular speed can come from odometry,
    joint_state or imu messages  

- ~minimal_gps_rate(float, default: 1) 

    Miminal rate of gps fix.   

- ~minimal_course_rate(float, default: 1) 

    Miminal rate of gps course .   

- ~maximal_dead_recknoning_travelled_distance(float, default=2)

   Maximal distance travelled in dead reckoning mode

- ~maximal_dead_recknoning_elapsed_time(float, default=10.)

   Maximal elapsed time (in second) in dead reckoning mode
   
- ~autostart (bool, default: false)

    Start or not localisation processing 
    
- ~debug (bool, default: false)

    Eanble or not rviz display and logs


### 2.5 Service server ###

- fsm_service (romea_lifecycle_msgs::FSMService)

    When control is done by fsm state machine. Some services can be called via   
    fsm_service topic (see romea_lifecycle_msgs for more information about this topic).   
    Available fsm_service resquests are:  
    &nbsp;&nbsp;&nbsp;&nbsp;-start : start localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-stop : stop localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-reset : reset localisation  
    &nbsp;&nbsp;&nbsp;&nbsp;-checkStatus : check value of localisation status   
