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
from launch_ros.actions import SetParameter, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_common_bringup import load_configuration


def additional_launch_arguments(gps_plugin_configuration):
    plugin_configuration = gps_plugin_configuration.get("configuration", None)

    if plugin_configuration is not None:
        return dict(
            zip(
                plugin_configuration.keys(),
                [str(value) for value in plugin_configuration.values()],
            )
        )
    else:
        return {}


def get_sensor_meta_description_filename(gps_plugin_configuration):
    return gps_plugin_configuration["input_sensor_meta_description_filename"]


def get_sensors_meta_description_filenames(gps_plugin_configuration):
    return gps_plugin_configuration["input_sensors_meta_description_filenames"]


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_localisation_configuration(context):
    return load_configuration(
        LaunchConfiguration("localisation_configuration_file_path").perform(context)
    )


def get_base_meta_description_file_path(context):
    return LaunchConfiguration("base_meta_description_file_path").perform(context)


def get_sensors_meta_descriptions_file_paths(context):
    return yaml.safe_load(
        LaunchConfiguration("sensors_meta_description_file_paths").perform(context)
    )


def get_leader_namespace(context):
    return LaunchConfiguration("leader_namespace").perform(context)


def get_leader_transceivers_meta_description_file_paths(context):
    return yaml.safe_load(
        LaunchConfiguration("leader_transceivers_meta_description_file_paths").perform(context)
    )


def get_sensor_meta_description_file_path(context, sensor_meta_description_filename):

    sensors_meta_description_file_paths = get_sensors_meta_descriptions_file_paths(context)

    sensor_meta_description_file_path = next(
        (
            sensor_meta_description_file_path
            for sensor_meta_description_file_path in sensors_meta_description_file_paths
            if sensor_meta_description_filename in sensor_meta_description_file_path
        ),
        None,
    )

    return sensor_meta_description_file_path


def get_sensors_meta_description_file_paths(context, sensors_meta_description_filenames):

    return [
        get_sensor_meta_description_file_path(context, meta_description_filename)
        for meta_description_filename in sensors_meta_description_filenames
    ]


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)
    localisation_configuration = get_localisation_configuration(context)
    container = ""

    actions = [
        SetParameter(name="use_sim_time", value=(mode != "live")),
        PushRosNamespace(robot_namespace),
        PushRosNamespace("localisation"),
    ]

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]

    launch_arguments = {
        "mode": mode,
        "robot_namespace": robot_namespace,
        "base_meta_description_file_path": get_base_meta_description_file_path(context),
        "launch_file": odo_plugin_configuration["launch"],
        "component_container": container,

    } | additional_launch_arguments(odo_plugin_configuration)

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/plugins/"
                + odo_plugin_configuration["pkg"]
                + ".launch.py"
            ),
            launch_arguments=launch_arguments.items(),
        )
    )

    if "imu" in localisation_configuration["plugins"]:
        imu_plugin_configuration = localisation_configuration["plugins"]["imu"]

        imu_meta_description_file_path = get_sensor_meta_description_file_path(
            context, get_sensor_meta_description_filename(imu_plugin_configuration)
        )

        launch_arguments = {
            "mode": mode,
            "robot_namespace": robot_namespace,
            "base_meta_description_file_path": get_base_meta_description_file_path(context),
            "imu_meta_description_file_path": imu_meta_description_file_path,
            "launch_file": imu_plugin_configuration["launch"],
            "component_container": container,
        } | additional_launch_arguments(imu_plugin_configuration)

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_localisation_bringup")
                    + "/launch/plugins/" + imu_plugin_configuration["pkg"] + ".launch.py"
                ),
                launch_arguments=launch_arguments.items(),
            )
        )

    rtls_plugin_configuration = localisation_configuration["plugins"]["rtls"]

    leader_namespace = get_leader_namespace(context)

    leader_transceivers_meta_description_file_paths = (
        get_leader_transceivers_meta_description_file_paths(context)
    )

    robot_transceivers_meta_description_file_paths = get_sensors_meta_description_file_paths(
        context, get_sensors_meta_description_filenames(rtls_plugin_configuration),
    )

    launch_arguments = {
        "mode": mode,
        "initiators_namespace": robot_namespace,
        "initiators_meta_description_file_paths": str(
            robot_transceivers_meta_description_file_paths
        ),
        "responders_namespace": leader_namespace,
        "responders_meta_description_file_paths": str(
            leader_transceivers_meta_description_file_paths
        ),
        "plugin_package": rtls_plugin_configuration["pkg"],
        "launch_file": rtls_plugin_configuration["launch"],
        "component_container": container,
    } | additional_launch_arguments(rtls_plugin_configuration)

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/plugins/romea_localisation_rtls_plugin.launch.py"
            ),
            launch_arguments=launch_arguments.items(),
        )
    )

    core_configuration = localisation_configuration["core"]

    launch_arguments = {
        "robot_namespace": robot_namespace,
        "has_imu_plugin": str("imu" in localisation_configuration["plugins"]),
        "component_container": container,
        "launch_file": core_configuration["launch"],
    } | additional_launch_arguments(core_configuration)

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/cores/"
                + core_configuration["pkg"]
                + ".launch.py"
            ),
            launch_arguments=launch_arguments.items(),
        )
    )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("localisation_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("sensors_meta_description_file_paths"))

    declared_arguments.append(DeclareLaunchArgument("leader_namespace"))

    declared_arguments.append(
        DeclareLaunchArgument("leader_transceivers_meta_description_file_paths")
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
