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
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_common_bringup import save_configuration, load_meta_descriptions
from romea_rtls_transceiver_bringup import (
    get_transceivers_ros_names,
    get_transceivers_ids,
    get_transceivers_xyz,
)


def get_mode(context):
    return LaunchConfiguration("mode").perform(context)


def get_plugin_package(context):
    return LaunchConfiguration("plugin_package").perform(context)


def get_controller_topic(context):
    return LaunchConfiguration("controller_topic").perform(context)


def get_poll_rate(context):
    return float(LaunchConfiguration("poll_rate").perform(context))


def get_range_std(context):
    return float(LaunchConfiguration("range_std").perform(context))


def get_minimal_range(context):
    return float(LaunchConfiguration("minimal_range").perform(context))


def get_maximal_range(context):
    return float(LaunchConfiguration("maximal_range").perform(context))


def get_rx_power_outlier_rejection_threshold(context):
    return int(LaunchConfiguration("rx_power_outlier_rejection_threshold").perform(context))


def get_initiators_namespace(context):
    return LaunchConfiguration("initiators_namespace").perform(context)


def get_initiators_meta_descriptions(context):
    return load_meta_descriptions(
        yaml.safe_load(
            LaunchConfiguration("initiators_meta_description_file_paths").perform(context)
        )
    )


def get_responders_namespace(context):
    return LaunchConfiguration("responders_namespace").perform(context)


def get_responders_meta_descriptions(context):
    return load_meta_descriptions(
        yaml.safe_load(
            LaunchConfiguration("responders_meta_description_file_paths").perform(context)
        )
    )


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def launch_setup(context, *args, **kwargs):

    mode = get_mode(context)
    initiators_namespace = get_initiators_namespace(context)
    initiators_meta_descriptions = get_initiators_meta_descriptions(context)
    responders_namespace = get_responders_namespace(context)
    responders_meta_descriptions = get_responders_meta_descriptions(context)

    initiators_names = get_transceivers_ros_names(
        initiators_namespace, initiators_meta_descriptions
    )
    initiators_ids = get_transceivers_ids(initiators_meta_descriptions)
    initiators_xyz = get_transceivers_xyz(initiators_meta_descriptions)
    responders_names = get_transceivers_ros_names(
        responders_namespace, responders_meta_descriptions
    )
    responders_ids = get_transceivers_ids(responders_meta_descriptions)
    responders_xyz = get_transceivers_xyz(responders_meta_descriptions)

    configuration = {}
    configuration["enable_scheduler"]: "replay" not in mode  # noqa: F821
    configuration["poll_rate"] = get_poll_rate(context)
    configuration["minimal_range"] = get_minimal_range(context)
    configuration["maximal_range"] = get_maximal_range(context)
    configuration["range_std"] = get_range_std(context)
    configuration[
        "rx_power_outlier_rejection_threshold"
    ] = get_rx_power_outlier_rejection_threshold(context)

    if len(initiators_names) > 1:
        configuration["initiators_names"] = initiators_names
        configuration["initiators_ids"] = initiators_ids
        configuration["initiators_positions"] = dict(
            zip(
                initiators_names,
                initiators_xyz,
            )
        )
    else:
        configuration["initiator_id"] = initiators_ids[0]
        configuration["initiator_name"] = initiators_names[0]
        configuration["initiator_position"] = initiators_xyz[0]

    if len(responders_names) > 1:
        configuration["responders_ids"] = responders_ids
        configuration["responders_names"] = responders_names
        configuration["responders_positions"] = dict(
            zip(
                responders_names,
                responders_xyz,
            )
        )
    else:
        configuration["responder_id"] = responders_ids[0]
        configuration["responder_name"] = responders_names[0]
        configuration["responder_position"] = responders_xyz[0]

    configuration_file_path = "/tmp/" + initiators_namespace + "_localisation_rtls_plugin.yaml"
    save_configuration(configuration, configuration_file_path)

    plugin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory(get_plugin_package(context))
            + "/launch/rtls_plugin.launch.py"
        ),
        launch_arguments={
            "component_container": get_component_container(context),
            "plugin_configuration_file_path": configuration_file_path,
        }.items(),
    )

    return [plugin]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("mode"))

    declared_arguments.append(DeclareLaunchArgument("initiators_namespace"))

    declared_arguments.append(DeclareLaunchArgument("initiators_meta_description_file_paths"))

    declared_arguments.append(DeclareLaunchArgument("plugin_package"))

    declared_arguments.append(DeclareLaunchArgument("poll_rate", default_value="20.0"))

    declared_arguments.append(DeclareLaunchArgument("range_std", default_value="0.1"))

    declared_arguments.append(DeclareLaunchArgument("minimal_range", default_value="0.5"))

    declared_arguments.append(DeclareLaunchArgument("maximal_range", default_value="20.0"))

    declared_arguments.append(
        DeclareLaunchArgument("rx_power_outlier_rejection_threshold", default_value="20")
    )

    declared_arguments.append(DeclareLaunchArgument("responders_meta_description_file_paths"))

    declared_arguments.append(DeclareLaunchArgument("component_container"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
