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

from romea_common_bringup import device_namespace, robot_prefix
from romea_imu_bringup import get_imu_specifications
from romea_gps_bringup import get_gps_specifications
# from romea_rtls_transceiver_bringup import RTLSTransceiverMetaDescription


def get_sensor_namespace(parent_namespace, sensor_meta_description):
    return device_namespace(
        parent_namespace,
        sensor_meta_description.get_namespace(),
        sensor_meta_description.get_name(),
    )


def get_controller_namespace(robot_namespace, odo_plugin_configuration):
    base_name = odo_plugin_configuration["base_name"]
    base_namespace = device_namespace(robot_namespace, None, base_name)
    return base_namespace + "/" + odo_plugin_configuration["controller_name"]


def get_filter_parameters(robot_namespace, core_configuration):
    base_footprint_frame_id = robot_prefix(robot_namespace) + "base_link"

    return [
        core_configuration,
        {"base_footprint_frame_id": base_footprint_frame_id},
    ]


def get_odo_plugin_parameters(mode, odo_plugin_configuration):
    return odo_plugin_configuration


def get_odo_plugin_remappings(robot_namespace, odo_plugin_configuration):

    controller_topic = odo_plugin_configuration["controller_topic"]
    controller_namespace = get_controller_namespace(
        robot_namespace, odo_plugin_configuration
    )

    return [
        (
            "vehicle_controller/" + controller_topic,
            controller_namespace + "/" + controller_topic,
        )
    ]


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


def get_imu_plugin_remappings(robot_namepsace, imu_plugin_configuration, odo_plugin_configuration):

    imu_namespace = get_sensor_namespace(
        robot_namepsace, imu_plugin_configuration)

    controller_namespace = get_controller_namespace(
        robot_namepsace, odo_plugin_configuration
    )

    return [
        ("imu/data", imu_namespace + "/data"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]


def get_gps_plugin_parameters(gps_plugin_configuration, gps_meta_description, wgs84_anchor):

    gps_specifications = get_gps_specifications(gps_meta_description)

    return [
        gps_plugin_configuration,
        {"wgs84_anchor": wgs84_anchor},
        {"gps": gps_specifications},
        {"gps.rate": float(gps_meta_description.get_rate())},
        {"gps.xyz": gps_meta_description.get_xyz()},
    ]


def get_gps_plugin_remappings(robot_namespace, gps_plugin_configuration, odo_plugin_configuration):

    gps_namespace = get_sensor_namespace(
        robot_namespace, gps_plugin_configuration)

    controller_namespace = get_controller_namespace(
        robot_namespace, odo_plugin_configuration
    )

    return [
        ("gps/nmea_sentence", gps_namespace + "/nmea_sentence"),
        ("vehicle_controller/odom", controller_namespace + "/odom"),
    ]
