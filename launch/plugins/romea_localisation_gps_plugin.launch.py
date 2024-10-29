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

import yaml

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    OpaqueFunction,
    GroupAction,
)

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_common_bringup import load_meta_description, save_configuration, load_configuration
from romea_localisation_bringup import get_device_namespace, get_controller_namespace
from romea_mobile_base_bringup import load_meta_description as load_base_description
from romea_gps_bringup import get_gps_specifications


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_base_meta_description(context):
    return load_base_description(
        LaunchConfiguration("base_meta_description_file_path").perform(context)
    )


def get_gps_meta_description(context):
    return load_meta_description(
        yaml.safe_load(LaunchConfiguration("gps_meta_description_file_path").perform(context))
    )


def get_wgs84_anchor(context):
    return load_configuration(LaunchConfiguration("wgs84_anchor_file_path").perform(context))


def get_minimal_speed_over_ground(context):
    return float(LaunchConfiguration("minimal_speed_over_ground").perform(context))


def get_minimal_fix_quality(context):
    return int(LaunchConfiguration("minimal_fix_quality").perform(context))


def get_restamping(context):
    return LaunchConfiguration("restamping").perform(context).lower() == "true"


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):

    # mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    base_meta_description = get_base_meta_description(context)
    gps_meta_description = get_gps_meta_description(context)

    configuration = {}
    configuration["wgs84_anchor"] = get_wgs84_anchor(context)
    configuration["gps"] = get_gps_specifications(gps_meta_description)
    configuration["gps"]["rate"] = float(gps_meta_description.get_rate())
    configuration["gps"]["xyz"] = gps_meta_description.get_xyz()
    configuration["minimal_speed_over_ground"] = get_minimal_speed_over_ground(context)
    configuration["minimal_fix_quality"] = get_minimal_fix_quality(context)
    configuration["restamping"] = get_restamping(context)

    configuration_file_path = "/tmp/"+robot_namespace+"_localisation_gps_plugin.yaml"
    save_configuration(configuration, configuration_file_path)

    actions = []

    gps_namespace = get_device_namespace(robot_namespace, gps_meta_description)
    actions.append(SetRemap(src="gps/nmea_sentence", dst=gps_namespace + "/nmea_sentence")),

    controller_namespace = get_controller_namespace(robot_namespace, base_meta_description)
    actions.append(SetRemap(src="vehicle_controller/odom", dst=controller_namespace + "/odom"))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_gps_plugin")
                + "/launch/gps_plugin.launch.py"
            ),
            launch_arguments={
                "component_container": get_component_container(context),
                "plugin_configuration_file_path": configuration_file_path,
                "dual_antenna": str(gps_meta_description.get_dual_antenna())
            }.items()
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("gps_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    declared_arguments.append(DeclareLaunchArgument("restamping", default_value="false"))

    declared_arguments.append(DeclareLaunchArgument("minimal_fix_quality", default_value="4"))

    declared_arguments.append(
        DeclareLaunchArgument("minimal_speed_over_ground", default_value="0.5")
    )

    declared_arguments.append(DeclareLaunchArgument("component_container"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
