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

from romea_mobile_base_bringup import load_meta_description as load_base_description
from romea_imu_bringup import get_imu_specifications

from romea_common_bringup import (
    load_meta_description,
    save_configuration,
    device_namespace,
)


def get_device_namespace(parent_namespace, sensor_meta_description):

    return device_namespace(
        parent_namespace,
        sensor_meta_description.get_namespace(),
        sensor_meta_description.get_name(),
    )


def get_controller_namespace(robot_namespace, base_meta_description):
    return get_device_namespace(robot_namespace, base_meta_description) + "/controller"


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_base_meta_description(context):
    return load_base_description(
        LaunchConfiguration("base_meta_description_file_path").perform(context)
    )


def get_imu_meta_description(context):
    return load_meta_description(
        yaml.safe_load(LaunchConfiguration("imu_meta_description_file_path").perform(context))
    )


def get_restamping(context):
    return bool(LaunchConfiguration("restamping").perform(context))


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    base_meta_description = get_base_meta_description(context)
    imu_meta_description = get_imu_meta_description(context)

    configuration = {}
    configuration["imu"] = get_imu_specifications(imu_meta_description)
    configuration["imu"]["rate"] = float(imu_meta_description.get_rate())
    configuration["imu"]["xyz"] = imu_meta_description.get_xyz()
    configuration["imu"]["rpy"] = imu_meta_description.get_rpy_rad()
    configuration["enable_accelerations"]: "live" in mode
    configuration["restamping"] = get_restamping(context)

    configuration_file_path = "/tmp/"+robot_namespace+"_localisation_imu_plugin.yaml"
    save_configuration(configuration, configuration_file_path)

    actions = []

    imu_namespace = get_device_namespace(robot_namespace, imu_meta_description)
    actions.append(SetRemap(src="imu/data", dst=imu_namespace + "/data")),

    controller_namespace = get_controller_namespace(robot_namespace, base_meta_description)
    actions.append(SetRemap(src="vehicle_controller/odom", dst=controller_namespace + "/odom"))

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_imu_plugin")
                + "/launch/imu_plugin.launch.py"
            ),
            launch_arguments={
                "component_container": get_component_container(context),
                "plugin_configuration_file_path": configuration_file_path,
            }.items()
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("imu_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("restamping", default_value="false"))

    declared_arguments.append(DeclareLaunchArgument("component_container"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
