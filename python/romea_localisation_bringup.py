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

from launch_ros.actions import Node
from romea_common_bringup import (
    device_namespace,
    robot_prefix,
    find_meta_description,
    find_meta_descriptions,
)
from romea_imu_bringup import get_imu_specifications
from romea_gps_bringup import get_gps_specifications
from romea_rtls_transceiver_bringup import (
    get_transceivers_ros_names,
    get_transceivers_ids,
    get_transceivers_xyz,
)


def get_device_namespace(parent_namespace, sensor_meta_description):

    return device_namespace(
        parent_namespace,
        sensor_meta_description.get_namespace(),
        sensor_meta_description.get_name(),
    )


def get_controller_namespace(robot_namespace, base_meta_description):
    return get_device_namespace(robot_namespace, base_meta_description) + "/controller"


def get_core_configuration(localisation_configuration):
    return localisation_configuration["core"]


def get_core_parameters(robot_namespace, core_configuration):
    base_footprint_frame_id = robot_prefix(robot_namespace) + "base_link"

    return [
        core_configuration,
        {"base_footprint_frame_id": base_footprint_frame_id},
    ]


def get_core(robot_namespace, filter_configuration):
    filter_parameters = get_core_parameters(robot_namespace, filter_configuration)

    return Node(
        package="romea_robot_to_world_localisation",
        executable="robot_to_world_kalman_localisation_node",
        name="robot_to_world_kalman_localisation",
        parameters=filter_parameters,
        output="screen",
    )


def get_odo_plugin_configuration(localisation_configuration):
    return localisation_configuration["plugins"]["odo"]["configuration"]


def get_odo_plugin_parameters(mode, odo_plugin_configuration):
    return [odo_plugin_configuration]


def get_odo_plugin_remappings(robot_namespace, odo_plugin_configuration, base_meta_description):

    controller_topic = odo_plugin_configuration["controller_topic"]
    controller_namespace = get_controller_namespace(robot_namespace, base_meta_description)

    return [
        (
            "vehicle_controller/" + controller_topic,
            controller_namespace + "/" + controller_topic,
        )
    ]


def get_odo_plugin(mode, robot_namespace, odo_plugin_configuration, base_meta_description):

    odo_plugin_parameters = get_odo_plugin_parameters(mode, odo_plugin_configuration)

    odo_plugin_remappings = get_odo_plugin_remappings(
        robot_namespace, odo_plugin_configuration, base_meta_description
    )
    return Node(
        package="romea_odo_localisation_plugin",
        executable="odo_localisation_plugin_node",
        name="odo_localisation_plugin",
        parameters=odo_plugin_parameters,
        remappings=odo_plugin_remappings,
        output="screen",
    )


def get_imu_plugin_configuration(localisation_configurations):
    return localisation_configurations["plugins"]["imu"]["configuration"]


def get_imu_meta_description(localisation_configurations, robot_sensors_meta_descriptions):
    return find_meta_description(
        robot_sensors_meta_descriptions,
        localisation_configurations["plugins"]["imu"]["input_sensor_name"],
    )


def get_imu_plugin_parameters(mode, imu_plugin_configuration, imu_meta_description):

    imu_specifications = get_imu_specifications(imu_meta_description)

    return [
        imu_plugin_configuration,
        {"enable_accelerations": ("live" in mode)},
        {"imu": imu_specifications},
        {"imu.rate": float(imu_meta_description.get_rate())},
        {"imu.xyz": imu_meta_description.get_xyz()},
        {"imu.rpy": imu_meta_description.get_rpy_rad()},
    ]


def get_imu_plugin_remappings(robot_namepsace, imu_meta_description, base_meta_description):

    imu_namespace = get_device_namespace(robot_namepsace, imu_meta_description)

    controller_namespace = get_controller_namespace(robot_namepsace, base_meta_description)

    return [
        ("imu/data", imu_namespace + "/data"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]


def get_imu_plugin(
    mode,
    robot_namespace,
    imu_plugin_configuration,
    imu_meta_description,
    base_meta_description,
):

    imu_plugin_parameters = get_imu_plugin_parameters(
        mode, imu_plugin_configuration, imu_meta_description
    )

    imu_plugin_remappings = get_imu_plugin_remappings(
        robot_namespace, imu_meta_description, base_meta_description
    )

    return Node(
        package="romea_imu_localisation_plugin",
        executable="imu_localisation_plugin_node",
        name="imu_localisation_plugin",
        parameters=imu_plugin_parameters,
        remappings=imu_plugin_remappings,
        output="screen",
    )


def get_gps_plugin_configuration(localisation_configurations):
    return localisation_configurations["plugins"]["gps"]["configuration"]


def get_gps_meta_description(localisation_configurations, robot_sensors_meta_descriptions):
    return find_meta_description(
        robot_sensors_meta_descriptions,
        localisation_configurations["plugins"]["gps"]["input_sensor_name"],
    )


def get_gps_plugin_parameters(gps_plugin_configuration, gps_meta_description, wgs84_anchor):

    gps_specifications = get_gps_specifications(gps_meta_description)

    return [
        gps_plugin_configuration,
        {"wgs84_anchor": wgs84_anchor},
        {"gps": gps_specifications},
        {"gps.rate": float(gps_meta_description.get_rate())},
        {"gps.xyz": gps_meta_description.get_xyz()},
    ]


def get_gps_plugin_remappings(robot_namespace, gps_meta_description, base_meta_description):

    gps_namespace = get_device_namespace(robot_namespace, gps_meta_description)

    controller_namespace = get_controller_namespace(robot_namespace, base_meta_description)

    return [
        ("gps/nmea_sentence", gps_namespace + "/nmea_sentence"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]


def get_gps_plugin(
    mode,
    robot_namespace,
    gps_plugin_configuration,
    gps_meta_description,
    odo_plugin_configuration,
    wgs84_anchor,
):

    gps_plugin_parameters = get_gps_plugin_parameters(
        gps_plugin_configuration, gps_meta_description, wgs84_anchor
    )

    gps_plugin_remappings = get_gps_plugin_remappings(
        robot_namespace, gps_meta_description, odo_plugin_configuration
    )

    return Node(
        package="romea_gps_localisation_plugin",
        executable="gps_localisation_plugin_node",
        name="gps_localisation_plugin",
        parameters=gps_plugin_parameters,
        remappings=gps_plugin_remappings,
        output="screen",
    )


def get_rtls_plugin_configuration(plugins_configurations):
    return plugins_configurations["plugins"]["rtls"]["configuration"]


def get_rtls_tranceivers_meta_descriptions(plugins_configurations, robot_sensors_meta_descriptions):
    return find_meta_descriptions(
        robot_sensors_meta_descriptions,
        plugins_configurations["plugins"]["rtls"]["input_sensors_names"],
    )


def get_rtls_plugin_parameters(
    mode,
    robot_namespace,
    rtls_plugin_configuration,
    initiators_meta_descriptions,
    responders_meta_descriptions,
):

    initiators_names = get_transceivers_ros_names(robot_namespace, initiators_meta_descriptions)
    initiators_ids = get_transceivers_ids(initiators_meta_descriptions)
    initiators_xyz = get_transceivers_xyz(initiators_meta_descriptions)
    responders_names = get_transceivers_ros_names("", responders_meta_descriptions)
    responders_ids = get_transceivers_ids(responders_meta_descriptions)
    responders_xyz = get_transceivers_xyz(responders_meta_descriptions)

    return [
        rtls_plugin_configuration,
        {"enable_scheduler": "replay" not in mode},
        {"initiators_names": initiators_names},
        {"initiators_ids": initiators_ids},
        {
            "initiators_positions": dict(
                zip(
                    initiators_names,
                    initiators_xyz,
                )
            )
        },
        {"responders_names": responders_names},
        {"responders_ids": responders_ids},
        {
            "responders_positions": dict(
                zip(
                    responders_names,
                    responders_xyz,
                )
            )
        },
    ]


def get_rtls_plugin(
    mode,
    robot_namespace,
    rtls_plugin_configuration,
    initiators_meta_descriptions,
    responders_meta_descriptions,
):

    rtls_plugin_parameters = get_rtls_plugin_parameters(
        mode,
        robot_namespace,
        rtls_plugin_configuration,
        initiators_meta_descriptions,
        responders_meta_descriptions,
    )

    return Node(
        package="romea_robot_to_world_localisation_rtls_plugin",
        executable="robot_to_world_rtls_localisation_plugin_node",
        name="rtls_localisation_plugin",
        parameters=rtls_plugin_parameters,
        # remappings=gps_plugin_remappings,
        output="screen",
    )
