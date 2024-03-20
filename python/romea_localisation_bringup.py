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

from romea_common_bringup import device_namespace


def get_device_namespace(parent_namespace, sensor_meta_description):

    return device_namespace(
        parent_namespace,
        sensor_meta_description.get_namespace(),
        sensor_meta_description.get_name(),
    )


def get_controller_namespace(robot_namespace, base_meta_description):
    return get_device_namespace(robot_namespace, base_meta_description) + "/controller"


def additional_launch_arguments(plugin_description):
    plugin_configuration = plugin_description.get("configuration", None)

    if plugin_configuration is not None:
        return dict(
            zip(
                plugin_configuration.keys(),
                [str(value) for value in plugin_configuration.values()],
            )
        )
    else:
        return {}


def get_sensor_meta_description_file_path(
    sensors_meta_description_file_paths, sensor_meta_description_filename
):

    sensor_meta_description_file_path = next(
        (
            sensor_meta_description_file_path
            for sensor_meta_description_file_path in sensors_meta_description_file_paths
            if sensor_meta_description_filename in sensor_meta_description_file_path
        ),
        None,
    )

    if not sensor_meta_description_file_path:
        raise LookupError(
            sensor_meta_description_filename
            + " meta description file not found."
            + " Available sensor meta descriptions are "
            + str(sensors_meta_description_file_paths)
        )

    return sensor_meta_description_file_path


def get_sensors_meta_description_file_paths(
    sensors_meta_description_file_paths, sensors_meta_description_filenames
):

    return [
        get_sensor_meta_description_file_path(
            sensors_meta_description_file_paths, meta_description_filename
        )
        for meta_description_filename in sensors_meta_description_filenames
    ]
