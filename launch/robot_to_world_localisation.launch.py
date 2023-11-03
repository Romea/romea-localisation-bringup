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
from romea_localisation_bringup import (
    additional_launch_arguments,
    get_sensor_meta_description_file_path,
    get_sensors_meta_description_file_paths
)


def get_sensor_meta_description_filename(gps_plugin_configuration):
    return gps_plugin_configuration["input_sensor_meta_description_filename"]


def get_sensors_meta_description_filenames(gps_plugin_configuration):
    return gps_plugin_configuration["input_sensors_meta_description_filenames"]


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_wgs84_anchor_file_path(context):
    return LaunchConfiguration("wgs84_anchor_file_path").perform(context)


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


def get_infrastructure_transceivers_meta_description_file_paths(context):
    return yaml.safe_load(
        LaunchConfiguration(
            "infrastructure_transceivers_meta_description_file_paths"
        ).perform(context)
    )


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

    core_configuration = localisation_configuration["core"]

    launch_arguments = {
        "robot_namespace": robot_namespace,
        "has_imu_plugin": str("imu" in localisation_configuration["plugins"]),
        "has_rtls_plugin": str("rtls" in localisation_configuration["plugins"]),
        "launch_file": core_configuration["launch"],
        "component_container": container,
    }.items() | additional_launch_arguments(core_configuration).items()

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/cores/"+core_configuration["pkg"]+".launch.py"
            ),
            launch_arguments=launch_arguments
        )
    )

    odo_plugin_configuration = localisation_configuration["plugins"]["odo"]

    launch_arguments = {
        "mode": mode,
        "robot_namespace": robot_namespace,
        "base_meta_description_file_path": get_base_meta_description_file_path(context),
        "launch_file": odo_plugin_configuration["launch"],
        "component_container": container,
    }.items() | additional_launch_arguments(odo_plugin_configuration).items()

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/plugins/"+odo_plugin_configuration["pkg"]+".launch.py"
            ),
            launch_arguments=launch_arguments
        )
    )

    if "imu" in localisation_configuration["plugins"]:
        imu_plugin_configuration = localisation_configuration["plugins"]["imu"]

        launch_arguments = {
            "mode": mode,
            "robot_namespace": robot_namespace,
            "base_meta_description_file_path": get_base_meta_description_file_path(context),
            "imu_meta_description_file_path": get_sensor_meta_description_file_path(
                get_sensors_meta_descriptions_file_paths(context),
                get_sensor_meta_description_filename(imu_plugin_configuration)
            ),
            "launch_file": imu_plugin_configuration["launch"],
            "component_container": container,
        }.items() | additional_launch_arguments(imu_plugin_configuration).items()

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_localisation_bringup")
                    + "/launch/plugins/"+imu_plugin_configuration["pkg"]+".launch.py"
                ),
                launch_arguments=launch_arguments
            )
        )

    gps_plugin_configuration = localisation_configuration["plugins"]["gps"]

    launch_arguments = {
        "mode": mode,
        "robot_namespace": robot_namespace,
        "wgs84_anchor_file_path": get_wgs84_anchor_file_path(context),
        "base_meta_description_file_path": get_base_meta_description_file_path(context),
        "gps_meta_description_file_path": get_sensor_meta_description_file_path(
            get_sensors_meta_descriptions_file_paths(context),
            get_sensor_meta_description_filename(gps_plugin_configuration)
        ),
        "launch_file": gps_plugin_configuration["launch"],
        "component_container": container,
    }.items() | additional_launch_arguments(gps_plugin_configuration).items()

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory("romea_localisation_bringup")
                + "/launch/plugins/"+gps_plugin_configuration["pkg"]+".launch.py"
            ),
            launch_arguments=launch_arguments
        )
    )

    if "rtls" in localisation_configuration["plugins"]:
        rtls_plugin_configuration = localisation_configuration["plugins"]["rtls"]

        launch_arguments = {
            "mode": mode,
            "initiators_namespace": robot_namespace,
            "initiators_meta_description_file_paths": str(
                get_sensors_meta_description_file_paths(
                    get_sensors_meta_descriptions_file_paths(context),
                    get_sensors_meta_description_filenames(rtls_plugin_configuration),
                )
            ),
            "responders_namespace": "",
            "responders_meta_description_file_paths": str(
                get_infrastructure_transceivers_meta_description_file_paths(context)
            ),
            "plugin_package": rtls_plugin_configuration["pkg"],
            "launch_file": rtls_plugin_configuration["launch"],
            "component_container": container,
        }.items() | additional_launch_arguments(rtls_plugin_configuration).items()

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    get_package_share_directory("romea_localisation_bringup")
                    + "/launch/plugins/romea_localisation_rtls_plugin.launch.py"
                ),
                launch_arguments=launch_arguments,
            )
        )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("localisation_configuration_file_path")
    )

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(
        DeclareLaunchArgument("sensors_meta_description_file_paths")
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "infrastructure_transceivers_meta_description_file_paths",
            default_value="[]",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
