# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from enum import Enum
from collections import deque
import random

import carla
from agents.navigation.controller import VehiclePIDController


class RoadOption(Enum):
    # RoadOption, bir şerit segmentinden diğerine geçerken olası topolojik konfigürasyonları temsil eder.
    
    VOID = -1
    LEFT = 1
    RIGHT = 2
    STRAIGHT = 3
    LANEFOLLOW = 4
    CHANGELANELEFT = 5
    CHANGELANERIGHT = 6


class LocalPlanner(object):
    # LocalPlanner, hareket halindeyken oluşturulan yol noktalarının
    # yörüngesini takip etme temel davranışını uygular. Aracın düşük seviyeli hareketi,
    # biri enine kontrol ve diğeri boyuna kontrol (seyir hızı) için kullanılan 
    # iki PID kontrolör kullanılarak hesaplanır.

    # Birden fazla yol mevcut olduğunda (kavşaklar), bu yerel planlayıcı 
    # rastgele bir seçim yapar.

    # Yüzde olarak hedef yol noktasına olan minimum mesafe (ör. toplam mesafenin % 90'ı içinde)
    MIN_DISTANCE_PERCENTAGE = 0.9

    def __init__(self, vehicle, opt_dict=None):
        self._vehicle = vehicle
        self._world = self._vehicle.get_world()
        self._map = self._world.get_map()

        self._dt = None
        self._target_speed = None
        self._sampling_radius = None
        self._min_distance = None
        self._current_waypoint = None
        self._target_road_option = None
        self._next_waypoints = None
        self.target_waypoint = None
        self._vehicle_controller = None
        self._global_plan = None
        # (waypoint, RoadOption) tuple kuyruğu
        self._waypoints_queue = deque(maxlen=20000)
        self._buffer_size = 5
        self._waypoint_buffer = deque(maxlen=self._buffer_size)

        # Kontrolcü başlatılır
        self._init_controller(opt_dict)

    def __del__(self):
        if self._vehicle:
            self._vehicle.destroy()
            print("Destroying ego-vehicle!")

    def reset_vehicle(self):
        self._vehicle = None
        print("Resetting ego-vehicle!")

    def _init_controller(self, opt_dict):
        # Kontrolcü başlatılır.

        # Varsayılan parametreler
        self._dt = 1.0 / 20.0
        self._target_speed = 20.0  # Km/h
        self._sampling_radius = self._target_speed * 1 / 3.6 
        self._min_distance = self._sampling_radius * self.MIN_DISTANCE_PERCENTAGE
        self._max_brake = 0.3
        self._max_throt = 0.75
        self._max_steer = 0.8
        args_lateral_dict = {
            'K_P': 1.95,
            'K_D': 0.2,
            'K_I': 0.07,
            'dt': self._dt}
        args_longitudinal_dict = {
            'K_P': 1.0,
            'K_D': 0,
            'K_I': 0.05,
            'dt': self._dt}

        # Parametrelerin aşırı yüklenmesi
        if opt_dict:
            if 'dt' in opt_dict:
                self._dt = opt_dict['dt']
            if 'target_speed' in opt_dict:
                self._target_speed = opt_dict['target_speed']
            if 'sampling_radius' in opt_dict:
                self._sampling_radius = self._target_speed * \
                                        opt_dict['sampling_radius'] / 3.6
            if 'lateral_control_dict' in opt_dict:
                args_lateral_dict = opt_dict['lateral_control_dict']
            if 'longitudinal_control_dict' in opt_dict:
                args_longitudinal_dict = opt_dict['longitudinal_control_dict']
            if 'max_throttle' in opt_dict:
                self._max_throt = opt_dict['max_throttle']
            if 'max_brake' in opt_dict:
                self._max_brake = opt_dict['max_brake']
            if 'max_steering' in opt_dict:
                self._max_steer = opt_dict['max_steering']

        self._current_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        self._vehicle_controller = VehiclePIDController(self._vehicle,
                                                        args_lateral=args_lateral_dict,
                                                        args_longitudinal=args_longitudinal_dict,
                                                        max_throttle=self._max_throt,
                                                        max_brake=self._max_brake,
                                                        max_steering=self._max_steer,)

        self._global_plan = False

        # İlk ara noktalar hesaplanır
        self._waypoints_queue.append((self._current_waypoint.next(self._sampling_radius)[0], RoadOption.LANEFOLLOW))

        self._target_road_option = RoadOption.LANEFOLLOW
        # Yol noktası yörünge kuyruğunu doldurur.
        self._compute_next_waypoints(k=200)

    def set_speed(self, speed):
        # Yeni bir hedef hız ayarlar.
        
        self._target_speed = speed

    def _compute_next_waypoints(self, k=1):
        # Yörünge kuyruğuna yeni ara noktalar ekler.
    
        # Kuyruğun aşılmadığı kontrol edilir
        available_entries = self._waypoints_queue.maxlen - len(self._waypoints_queue)
        k = min(available_entries, k)

        for _ in range(k):
            last_waypoint = self._waypoints_queue[-1][0]
            next_waypoints = list(last_waypoint.next(self._sampling_radius))

            if len(next_waypoints) == 0:
                break
            elif len(next_waypoints) == 1:
                # Sadece bir seçenek mevcut ==> lanefollowing
                next_waypoint = next_waypoints[0]
                road_option = RoadOption.LANEFOLLOW
            else:
                # Olası seçenekler arasında rastgele seçim yapılır
                road_options_list = _retrieve_options(
                    next_waypoints, last_waypoint)
                road_option = random.choice(road_options_list)
                next_waypoint = next_waypoints[road_options_list.index(
                    road_option)]

            self._waypoints_queue.append((next_waypoint, road_option))

    def set_global_plan(self, current_plan):
        # Yol noktası kuyruğunu ve arabelleği yeni plana uyacak şekilde sıfırlar. 
        # Ayrıca, daha fazla ara nokta oluşturmaktan kaçınmak için global_plan bayrağını ayarlar

        # Kuyruk sıfırlanır
        self._waypoints_queue.clear()
        for elem in current_plan:
            self._waypoints_queue.append(elem)
        self._target_road_option = RoadOption.LANEFOLLOW

        # Arabellek sıfırlanır
        self._waypoint_buffer.clear()
        for _ in range(self._buffer_size):
            if self._waypoints_queue:
                self._waypoint_buffer.append(
                    self._waypoints_queue.popleft())
            else:
                break

        self._global_plan = True

    def run_step(self, debug=False):
        # Yol noktalarının yörüngesini izlemek için boyuna ve 
        # enine PID kontrolörlerini çalıştırmayı içeren bir yerel planlama adımını gerçekleştirin.

        # Yeterli ara nokta yok mu? => daha fazla ekle!
        if not self._global_plan and len(self._waypoints_queue) < int(self._waypoints_queue.maxlen * 0.5):
            self._compute_next_waypoints(k=100)

        if len(self._waypoints_queue) == 0 and len(self._waypoint_buffer) == 0:
            control = carla.VehicleControl()
            control.steer = 0.0
            control.throttle = 0.0
            control.brake = 1.0
            control.hand_brake = False
            control.manual_gear_shift = False

            return control

        # Yol noktalarının korunması
        if not self._waypoint_buffer:
            for _ in range(self._buffer_size):
                if self._waypoints_queue:
                    self._waypoint_buffer.append(
                        self._waypoints_queue.popleft())
                else:
                    break

        # mevcut araç yol noktası
        vehicle_transform = self._vehicle.get_transform()
        self._current_waypoint = self._map.get_waypoint(vehicle_transform.location)
        # hedef yol noktası
        self.target_waypoint, self._target_road_option = self._waypoint_buffer[0]
        # PID kontrolcüleri kullanılarak hareket ettilir
        control = self._vehicle_controller.run_step(self._target_speed, self.target_waypoint)

        # önceki ara noktaların kuyruğu temizlenir
        max_index = -1

        for i, (waypoint, _) in enumerate(self._waypoint_buffer):
            if waypoint.transform.location.distance(vehicle_transform.location) < self._min_distance:
                max_index = i
        if max_index >= 0:
            for i in range(max_index + 1):
                self._waypoint_buffer.popleft()

        return control

    def done(self):
        # Planlayıcının bitirip bitirmediğini döndürür.
        
        return len(self._waypoints_queue) == 0 and len(self._waypoint_buffer) == 0

    def get_target_road_option(self):
        # Mevcut yol seçeneğini döndürür.
        
        return self._target_road_option

def _retrieve_options(list_waypoints, current_waypoint):
    # Mevcut aktif yol noktası ile list_waypoints'te bulunan çoklu yol noktaları arasındaki bağlantı türü hesaplanır. 
    # Sonuç, RoadOption numaralandırmalarının bir listesi olarak kodlanır.
    
    options = []
    for next_waypoint in list_waypoints:
        next_next_waypoint = next_waypoint.next(3.0)[0]
        link = _compute_connection(current_waypoint, next_next_waypoint)
        options.append(link)

    return options


def _compute_connection(current_waypoint, next_waypoint, threshold=35):
    # Etkin bir yol noktası(geçerli yol noktası) ile 
    # hedef yol noktası(sonraki_ yol noktası) arasındaki topolojik bağlantı türü hesaplanır.
    
    n = next_waypoint.transform.rotation.yaw
    n = n % 360.0

    c = current_waypoint.transform.rotation.yaw
    c = c % 360.0

    diff_angle = (n - c) % 180.0
    if diff_angle < threshold or diff_angle > (180 - threshold):
        return RoadOption.STRAIGHT
    elif diff_angle > 90.0:
        return RoadOption.LEFT
    else:
        return RoadOption.RIGHT
