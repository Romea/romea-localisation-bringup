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
from launch.actions import DeclareLaunchArgument, OpaqueFunction, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter, PushRosNamespace

from romea_mobile_base_bringup import load_meta_description as load_base_description
from romea_common_bringup import (
    load_meta_descriptions as load_sensors_meta_descriptions,
)
from romea_common_bringup import load_configuration

from romea_localisation_bringup import (
    get_core,
    get_core_configuration,
    get_odo_plugin,
    get_odo_plugin_configuration,
    get_imu_plugin,
    get_imu_plugin_configuration,
    get_imu_meta_description,
    get_gps_plugin,
    get_gps_plugin_configuration,
    get_gps_meta_description,
    get_rtls_plugin,
    get_rtls_plugin_configuration,
    get_rtls_tranceivers_meta_descriptions,
)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_wgs84_anchor(context):
    return load_configuration(LaunchConfiguration("wgs84_anchor_file_path").perform(context))


def get_localisation_configuration(context):
    return load_configuration(
        LaunchConfiguration("localisation_configuration_file_path").perform(context)
    )


def get_base_meta_description(context):
    return load_base_description(
        LaunchConfiguration("base_meta_description_file_path").perform(context)
    )


def get_sensors_meta_descriptions(context):
    return load_sensors_meta_descriptions(
        yaml.safe_load(LaunchConfiguration("sensors_meta_description_file_paths").perform(context))
    )


def get_infrastructure_transceivers_meta_descriptions(context):
    return load_sensors_meta_descriptions(
        yaml.safe_load(
            LaunchConfiguration("infrastructure_transceivers_meta_description_file_paths").perform(
                context
            )
        )
    )


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    robot_namespace = get_robot_namespace(context)

    wgs84_anchor = get_wgs84_anchor(context)
    localisation_configuration = get_localisation_configuration(context)
    sensors_meta_descriptions = get_sensors_meta_descriptions(context)
    base_meta_description = get_base_meta_description(context)

    infrastructure_transceivers_meta_descriptions = (
        get_infrastructure_transceivers_meta_descriptions(context)
    )

    # TODO replace code below by a composition

    actions = [
        SetParameter(name="use_sim_time", value=(mode != "live")),
        PushRosNamespace(robot_namespace),
        PushRosNamespace("localisation"),
    ]

    core_configuration = get_core_configuration(localisation_configuration)

    actions.append(get_core(robot_namespace, core_configuration))

    odo_plugin_configuration = get_odo_plugin_configuration(localisation_configuration)

    actions.append(
        get_odo_plugin(mode, robot_namespace, odo_plugin_configuration, base_meta_description)
    )

    imu_meta_description = get_imu_meta_description(
        localisation_configuration, sensors_meta_descriptions
    )

    imu_plugin_configuration = get_imu_plugin_configuration(localisation_configuration)

    actions.append(
        get_imu_plugin(
            mode,
            robot_namespace,
            imu_plugin_configuration,
            imu_meta_description,
            base_meta_description,
        )
    )

    gps_meta_description = get_gps_meta_description(
        localisation_configuration, sensors_meta_descriptions
    )

    gps_plugin_configuration = get_gps_plugin_configuration(localisation_configuration)

    actions.append(
        get_gps_plugin(
            mode,
            robot_namespace,
            gps_plugin_configuration,
            gps_meta_description,
            base_meta_description,
            wgs84_anchor,
        )
    )

    if "rtls" in localisation_configuration["plugins"] and len(
        infrastructure_transceivers_meta_descriptions
    ):

        rtls_tranceivers_meta_descriptions = get_rtls_tranceivers_meta_descriptions(
            localisation_configuration, sensors_meta_descriptions
        )

        rtls_plugin_configuration = get_rtls_plugin_configuration(localisation_configuration)

        actions.append(
            get_rtls_plugin(
                mode,
                robot_namespace,
                rtls_plugin_configuration,
                rtls_tranceivers_meta_descriptions,
                infrastructure_transceivers_meta_descriptions,
            )
        )

    return [GroupAction(actions)]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(DeclareLaunchArgument("wgs84_anchor_file_path"))

    declared_arguments.append(DeclareLaunchArgument("localisation_configuration_file_path"))

    declared_arguments.append(DeclareLaunchArgument("base_meta_description_file_path"))

    declared_arguments.append(DeclareLaunchArgument("sensors_meta_description_file_paths"))

    declared_arguments.append(
        DeclareLaunchArgument(
            "infrastructure_transceivers_meta_description_file_paths", default_value="[]"
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
