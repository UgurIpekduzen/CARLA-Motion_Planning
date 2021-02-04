#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import numpy as np
import carla

actor_list = []
collision_hist = []

def set_vehicle(_world, transform):
    # Aracın kurulumu yapılır
    blueprint_library = _world.get_blueprint_library()
    _vehicle = _world.spawn_actor(blueprint_library.filter("model3")[0],
                                    transform
                                    )
    actor_list.append(_vehicle)
    print("Actor " + str(_vehicle.id) + " added at " + str(transform.location))

    # Çarpışma sensörü
    _col_sensor = blueprint_library.find("sensor.other.collision")
    _col_sensor = _world.spawn_actor(_col_sensor, transform, attach_to=_vehicle)
    actor_list.append(_col_sensor)
    _col_sensor.listen(lambda event: _collision_data(event))
    print("Collision sensor " + str(_col_sensor.id) + " added")
    
    return _vehicle

def _collision_data(event):
    # Eğer bir çarpışma gerçekleşirse çarpışmanın olduğu konum kaydedilir.
    print("COLLISION HAPPENED")
    event_loc = event.other_actor.get_location()
    return collision_hist.append((event_loc.x, event_loc.y)) 

# Mevcut harita üzerindeki herhangi bir nesneyi işaretler.
def draw_string(world, object_location, color, string, life_time):
    world.debug.draw_string(object_location, string, draw_shadow=False,
                                       color=carla.Color(r=color[0], g=color[1], b=color[2]), life_time=life_time,
                                       persistent_lines=True)

def draw_waypoints(world, waypoints, z=0.5):
    # Z ile verilen belirli bir yükseklikteki ara noktaların bir listesi çizilir.
    
    for wpt in waypoints:
        wpt_t = wpt.transform
        begin = wpt_t.location + carla.Location(z=z)
        angle = math.radians(wpt_t.rotation.yaw)
        end = begin + carla.Location(x=math.cos(angle), y=math.sin(angle))
        world.debug.draw_arrow(begin, end, arrow_size=0.3, life_time=1.0)


def get_speed(vehicle):
    # Bir aracın hızını Km / saat cinsinden hesaplar.
    
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)

def is_within_distance_ahead(target_transform, current_transform, max_distance):
    # Hedef nesnenin, referans nesnenin önünde belirli bir mesafe içinde olup olmadığı kontrol edilir.
    
    target_vector = np.array([target_transform.location.x - current_transform.location.x, target_transform.location.y - current_transform.location.y])
    norm_target = np.linalg.norm(target_vector)

    # Vektör çok kısaysa, burada durdurulabilir
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    fwd = current_transform.get_forward_vector()
    forward_vector = np.array([fwd.x, fwd.y])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle < 90.0

def is_within_distance(target_location, current_location, orientation, max_distance, d_angle_th_up, d_angle_th_low=0):
    # Hedef nesnenin, referans nesneden belirli bir mesafede olup olmadığı kontrol edilir. 
    # Öndeki bir araç 0 derece civarında bir araç olursa, 180 derece civarında bir araç olacaktır.
    
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    # Vektör çok kısaysa, burada durdurulabilir
    if norm_target < 0.001:
        return True

    if norm_target > max_distance:
        return False

    forward_vector = np.array(
        [math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return d_angle_th_low < d_angle < d_angle_th_up


def compute_magnitude_angle(target_location, current_location, orientation):
    # Target_location ve current_location arasındaki göreceli açı ve mesafeyi hesaplar.
    
    target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
    norm_target = np.linalg.norm(target_vector)

    forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
    d_angle = math.degrees(math.acos(np.clip(np.dot(forward_vector, target_vector) / norm_target, -1., 1.)))

    return (norm_target, d_angle)


def distance_vehicle(waypoint, vehicle_transform):
    # Bir ara noktadan araca 2B mesafeyi verir.

    loc = vehicle_transform.location
    x = waypoint.transform.location.x - loc.x
    y = waypoint.transform.location.y - loc.y

    return math.sqrt(x * x + y * y)


def vector(location_1, location_2):
    # Birim vektörü location_1'den location_2'ye döndürür.

    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps

    return [x / norm, y / norm, z / norm]


def compute_distance(location_1, location_2):
    # 3B noktalar arasındaki Öklid mesafesi
    
    x = location_2.x - location_1.x
    y = location_2.y - location_1.y
    z = location_2.z - location_1.z
    norm = np.linalg.norm([x, y, z]) + np.finfo(float).eps
    return norm


def positive(num):
    # Pozitifse verilen sayıyı döndürür, yoksa 0 döndürür.
    
    return num if num > 0.0 else 0.0
