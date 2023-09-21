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

from romea_common_bringup import (
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


def get_controller_topic(context):
    return LaunchConfiguration("controller_topic").perform(context)


def get_restamping(context):
    return bool(LaunchConfiguration("restamping").perform(context))


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):

    # mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    base_meta_description = get_base_meta_description(context)

    configuration = {}
    configuration["controller_topic"] = get_controller_topic(context)
    configuration["restamping"] = get_restamping(context)

    configuration_file_path = (
        "/tmp/" + robot_namespace + "_localisation_odo_plugin.yaml"
    )
    save_configuration(configuration, configuration_file_path)

    actions = []

    controller_namespace = get_controller_namespace(
        robot_namespace, base_meta_description
    )

    actions.append(
        SetRemap(
            src="vehicle_controller/" + get_controller_topic(context),
            dst=controller_namespace + "/" + get_controller_topic(context),
        )
    )

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_odo_plugin")
                + "/launch/odo_plugin.launch.py"
            ),
            launch_arguments={
                "component_container": get_component_container(context),
                "plugin_configuration_file_path": configuration_file_path,
            }.items(),
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("restamping", default_value="false")
    )

    declared_arguments.append(
        DeclareLaunchArgument("controller_topic", default_value="kinematic")
    )

    declared_arguments.append(DeclareLaunchArgument("component_container"))

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
