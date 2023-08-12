# Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, PushRosNamespace

from romea_common_bringup import (
    load_meta_descriptions,
    find_meta_desription,
    load_configuration,
)

from romea_robot_to_world_localisation_bringup import (
    get_filter_parameters,
    get_odo_plugin_parameters,
    get_odo_plugin_remappings,
    get_imu_plugin_parameters,
    get_imu_plugin_remappings,
    get_gps_plugin_parameters,
    get_gps_plugin_remappings,
)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_wgs84_anchor(context):
    return load_configuration(
        LaunchConfiguration("wgs84_anchor_file_path").perform(context)
    )


def get_localisation_configuration(context):
    return load_configuration(
        LaunchConfiguration(
            "localisation_configuration_file_path").perform(context)
    )


def get_robot_sensor_meta_descriptions(context):
    return load_meta_descriptions(
        LaunchConfiguration("robot_sensors_meta_description_file_paths").perform(
            context
        )
    )


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    localisation_configuration = get_localisation_configuration(context)
    sensor_meta_descriptions = get_robot_sensor_meta_descriptions(context)
    wgs84_anchor = get_wgs84_anchor(context)

    filter_configuration = localisation_configuration["core"]

    filter_parameters = get_filter_parameters(
        robot_namespace, filter_configuration)

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]

    odo_plugin_parameters = get_odo_plugin_parameters(
        mode, odo_plugin_configuration)

    odo_plugin_remappings = get_odo_plugin_remappings(
        robot_namespace, odo_plugin_configuration
    )

    imu_plugin_configuration = localisation_configuration["plugins"]["imu"]

    imu_meta_description = find_meta_desription(
        sensor_meta_descriptions, imu_plugin_configuration["sensor_name"])

    imu_plugin_parameters = get_imu_plugin_parameters(
        mode, imu_plugin_configuration, imu_meta_description
    )

    imu_plugin_remappings = get_imu_plugin_remappings(
        robot_namespace, imu_plugin_configuration, odo_plugin_configuration
    )

    gps_plugin_configuration = localisation_configuration["plugins"]["gps"]

    gps_meta_description = find_meta_desription(
        sensor_meta_descriptions, gps_plugin_configuration["sensor_name"])

    gps_plugin_parameters = get_gps_plugin_parameters(
        mode, gps_plugin_configuration, gps_meta_description, wgs84_anchor
    )

    gps_plugin_remappings = get_gps_plugin_remappings(
        robot_namespace, gps_plugin_configuration
    )

    # TODO replace code below by a composition
    filter = Node(
        package="romea_robot_to_world_localisation",
        executable="robot_to_world_kalman_localisation_node",
        name="robot_to_world_kalman_localisation",
        parameters=filter_parameters,
        output="screen",
    )

    odo_plugin = Node(
        package="romea_odo_localisation_plugin",
        executable="odo_localisation_plugin_node",
        name="odo_localisation_plugin",
        parameters=odo_plugin_parameters,
        remappings=odo_plugin_remappings,
        output="screen",
    )

    imu_plugin = Node(
        package="romea_imu_localisation_plugin",
        executable="imu_localisation_plugin_node",
        name="imu_localisation_plugin",
        parameters=imu_plugin_parameters,
        remappings=imu_plugin_remappings,
        output="screen",
    )

    gps_plugin = Node(
        package="romea_gps_localisation_plugin",
        executable="gps_localisation_plugin_node",
        name="gps_localisation_plugin",
        parameters=gps_plugin_parameters,
        remappings=gps_plugin_remappings,
        output="screen",
    )

    return [
        GroupAction(
            actions=[
                SetParameter(name="use_sim_time", value=(mode != "live")),
                PushRosNamespace(robot_namespace),
                PushRosNamespace("localisation"),
                filter,
                odo_plugin,
                imu_plugin,
                gps_plugin,
            ]
        )
    ]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("robot_sensors_meta_description_file_paths")
    )

    declared_arguments.append(
        DeclareLaunchArgument("localisation_configuration_file_path")
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
