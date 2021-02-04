# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import sys
import math

from enum import Enum

import carla
from agents.tools.misc import is_within_distance_ahead, is_within_distance, compute_distance

class AgentState(Enum):
    
    # AGENT_STATE, bir dolaşım ajanının olası durumlarını temsil eder.
    
    NAVIGATING = 1
    BLOCKED_BY_VEHICLE = 2
    BLOCKED_RED_LIGHT = 3


class Agent(object):
    # CARLA'da ajanları tanımlamak için temel sınıf

    def __init__(self, vehicle):
        self._vehicle = vehicle
        self._proximity_tlight_threshold = 5.0  # meters
        self._proximity_vehicle_threshold = 10.0  # meters
        self._local_planner = None
        self._world = self._vehicle.get_world()
        try:
            self._map = self._world.get_map()
        except RuntimeError as error:
            print('RuntimeError: {}'.format(error))
            print('  The server could not send the OpenDRIVE (.xodr) file:')
            print('  Make sure it exists, has the same name of your town, and is correct.')
            sys.exit(1)
        self._last_traffic_light = None

    def get_local_planner(self):
        # Korunan üye yerel planlayıcı getirilir.
        return self._local_planner

    @staticmethod
    def run_step(debug=False):
        # Gezinmenin bir adımını yürütülür.
        
        control = carla.VehicleControl()
        
        if debug:
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 0.0
            control.hand_brake = False
            control.manual_gear_shift = False

        return control

    def _is_light_red(self, lights_list):
        # Aracı etkileyen bir kırmızı ışık olup olmadığı kontrol edilir. 
        # Fonksiyonun bu versiyonu hem Avrupa hem de ABD tarzı trafik ışıklarıyla uyumludur.
        
        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for traffic_light in lights_list:
            object_location = self._get_trafficlight_trigger_location(traffic_light)
            object_waypoint = self._map.get_waypoint(object_location)

            if object_waypoint.road_id != ego_vehicle_waypoint.road_id:
                continue

            ve_dir = ego_vehicle_waypoint.transform.get_forward_vector()
            wp_dir = object_waypoint.transform.get_forward_vector()
            dot_ve_wp = ve_dir.x * wp_dir.x + ve_dir.y * wp_dir.y + ve_dir.z * wp_dir.z

            if dot_ve_wp < 0:
                continue

            if is_within_distance_ahead(object_waypoint.transform,
                                        self._vehicle.get_transform(),
                                        self._proximity_tlight_threshold):
                if traffic_light.state == carla.TrafficLightState.Red:
                    return (True, traffic_light, object_location)

        return (False, None, None)

    def _get_trafficlight_trigger_location(self, traffic_light): 
        # Trafik ışığının tetikleme hacmini temsil eden yol noktasının sapmasını hesaplar.
        def rotate_point(point, radians):
            # Belirli bir noktayı belirli bir açıyla döndürür.
            
            rotated_x = math.cos(radians) * point.x - math.sin(radians) * point.y
            rotated_y = math.sin(radians) * point.x - math.cos(radians) * point.y

            return carla.Vector3D(rotated_x, rotated_y, point.z)

        base_transform = traffic_light.get_transform()
        base_rot = base_transform.rotation.yaw
        area_loc = base_transform.transform(traffic_light.trigger_volume.location)
        area_ext = traffic_light.trigger_volume.extent

        point = rotate_point(carla.Vector3D(0, 0, area_ext.z), math.radians(base_rot))
        point_location = area_loc + carla.Location(x=point.x, y=point.y)

        return carla.Location(point_location.x, point_location.y, point_location.z)

    def _is_vehicle_hazard(self, vehicle_list):
        # Aracı etkileyen bir başka araç olup olmadığı kontrol edilir.    

        ego_vehicle_location = self._vehicle.get_location()
        ego_vehicle_waypoint = self._map.get_waypoint(ego_vehicle_location)

        for target_vehicle in vehicle_list:
            # Ego aracı hesaba katılır
            if target_vehicle.id == self._vehicle.id:
                continue

            # Nesne şerit üzerinde değilse bu bir engel değildir
            target_vehicle_waypoint = self._map.get_waypoint(target_vehicle.get_location())
            if target_vehicle_waypoint.road_id != ego_vehicle_waypoint.road_id or \
                    target_vehicle_waypoint.lane_id != ego_vehicle_waypoint.lane_id:
                continue

            if is_within_distance_ahead(target_vehicle.get_transform(),
                                        self._vehicle.get_transform(),
                                        self._proximity_vehicle_threshold):
                return (True, target_vehicle)

        return (False, None)


    @staticmethod
    def emergency_stop():
        # Araca bir acil durdurma komutu gönderilir.

        control = carla.VehicleControl()
        control.steer = 0.0
        control.throttle = 0.0
        control.brake = 1.0
        control.hand_brake = False

        return control
