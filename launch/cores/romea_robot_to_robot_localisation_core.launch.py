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
)

from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

from romea_common_bringup import save_configuration


def get_robot_namespace(context):
    return LaunchConfiguration("robot_namespace").perform(context)


def get_core_type(context):
    return LaunchConfiguration("core_type").perform(context)


def get_filter_type(context):
    return LaunchConfiguration("filter_type").perform(context)


def get_debug(context):
    return bool(LaunchConfiguration("debug").perform(context))


def get_publish_rate(context):
    return int(LaunchConfiguration("publish_rate").perform(context))


def get_state_pool_size(context):
    return int(LaunchConfiguration("state_pool_size").perform(context))


def get_number_of_particles(context):
    return int(LaunchConfiguration("number_of_particles").perform(context))


def get_maximal_dead_recknoning_elapsed_time(context):
    return float(LaunchConfiguration("maximal_dead_recknoning_elapsed_time").perform(context))


def get_maximal_dead_recknoning_travelled_distance(context):
    return float(
        LaunchConfiguration("maximal_dead_recknoning_travelled_distance").perform(context)
    )


def has_imu_plugin(context):
    return bool(LaunchConfiguration("has_imu_plugin").perform(context))


def get_component_container(context):
    return LaunchConfiguration("component_container").perform(context)


def get_launch_file(context):
    return LaunchConfiguration("launch_file").perform(context)


def launch_setup(context, *args, **kwargs):

    robot_namespace = get_robot_namespace(context)

    state_pool_size = get_state_pool_size(context)
    number_of_particles = get_number_of_particles(context)
    maximal_dead_recknoning_elapsed_time = get_maximal_dead_recknoning_elapsed_time(context)
    maximal_dead_recknoning_travelled_distance = get_maximal_dead_recknoning_travelled_distance(
        context
    )

    configuration = {}
    configuration["filter"] = {
        "state_pool_size": state_pool_size,
        "number_of_particle": number_of_particles,
    }
    configuration["predictor"] = {
        "maximal_dead_recknoning_elapsed_time": maximal_dead_recknoning_elapsed_time,
        "maximal_dead_recknoning_travelled_distance": maximal_dead_recknoning_travelled_distance,
    }

    if has_imu_plugin(context):
        configuration["linear_speeds_updater"] = {"minimal_rate": 10}
        configuration["angular_speed_updater"] = {"minimal_rate": 10}
        configuration["attitude_updater"] = {"minimal_rate": 10}
    else:
        configuration["twist_updater"] = {"minimal_rate": 10}

    configuration["leader_twist_updater"] = {"minimal_rate": 10}
    configuration["pose_updater"] = {"minimal_rate": 1, "trigger": "once"}
    configuration["range_updater"] = {"minimal_rate": 10, "trigger": "always"}

    configuration["publish_rate"] = get_publish_rate(context)
    configuration["debug"] = get_debug(context)

    configuration_file_path = "/tmp/" + robot_namespace + "_robot_to_robot_localisation_core.yaml"
    save_configuration(configuration, configuration_file_path)

    core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory("romea_robot_to_robot_localisation_core")
            + "/launch/"
            + get_launch_file(context)
        ),
        launch_arguments={
            "filter_type": get_filter_type(context),
            "filter_configuration_file_path": configuration_file_path,
            "component_container": get_component_container(context),
        }.items(),
    )

    return [core]


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(DeclareLaunchArgument("robot_namespace"))

    declared_arguments.append(
        DeclareLaunchArgument("launch_file", default_value="robot_to_robot_localisation.launch.py")
    )

    declared_arguments.append(DeclareLaunchArgument("filter_type", default_value="kalman"))

    declared_arguments.append(DeclareLaunchArgument("state_pool_size", default_value="1000"))

    declared_arguments.append(DeclareLaunchArgument("number_of_particles", default_value="200"))

    declared_arguments.append(
        DeclareLaunchArgument("maximal_dead_recknoning_elapsed_time", default_value="10.")
    )

    declared_arguments.append(
        DeclareLaunchArgument("maximal_dead_recknoning_travelled_distance", default_value="2.")
    )

    declared_arguments.append(DeclareLaunchArgument("has_imu_plugin", default_value="true"))

    declared_arguments.append(DeclareLaunchArgument("has_rtls_plugin", default_value="false"))

    declared_arguments.append(DeclareLaunchArgument("publish_rate", default_value="10"))

    declared_arguments.append(DeclareLaunchArgument("debug", default_value="false"))

    declared_arguments.append(DeclareLaunchArgument("component_container"))

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
