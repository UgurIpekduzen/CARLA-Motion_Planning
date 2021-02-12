#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import math
import numpy as np
import carla
import cv2 
import os
import time
from natsort import natsorted

actor_list = []
collision_hist = []

IM_WIDTH = 640
IM_HEIGHT = 480
IMAGE_NO = 0
IMG_PATH = ""
FR_PATH = "./frames"
REC_PATH = "./records"

def set_vehicle(_world, transform):
    # Aracın kurulumu yapılır
    blueprint_library = _world.get_blueprint_library()
    _vehicle = _world.spawn_actor(blueprint_library.filter("model3")[0],
                                    transform
                                    )
    actor_list.append(_vehicle)
    print("Actor " + str(_vehicle.id) + " added at " + str(transform.location))

    # RGB Kamera
    rgb_cam = blueprint_library.find('sensor.camera.rgb')
    rgb_cam.set_attribute("image_size_x", f"{IM_WIDTH}")
    rgb_cam.set_attribute("image_size_y", f"{IM_HEIGHT}")
    rgb_cam.set_attribute("fov", f"110")

    transform = carla.Transform(carla.Location(x=2.5, z=0.7))
    _rgb_camera = _world.spawn_actor(rgb_cam, transform, attach_to=_vehicle, attachment_type=carla.AttachmentType.Rigid)
    actor_list.append(_rgb_camera)
    print("Front camera " + str(_rgb_camera.id) + " added")
    
    # Kamera görüntüleri için bir klasör oluşturulur
    try:
        os.makedirs(FR_PATH)
    except OSError:
        print ("Creation of the directory %s failed" % FR_PATH)
    else:
        print ("Successfully created the directory %s" % FR_PATH)

    # Çarpışma sensörü
    _col_sensor = blueprint_library.find("sensor.other.collision")
    _col_sensor = _world.spawn_actor(_col_sensor, transform, attach_to=_vehicle)
    actor_list.append(_col_sensor)
    print("Collision sensor " + str(_col_sensor.id) + " added")
    
    _rgb_camera.listen(lambda data: _process_img(data, _vehicle.get_control(), get_speed(_vehicle)))
    _col_sensor.listen(lambda event: _collision_data(event))
    
    return _vehicle

def _process_img(_image, control, speed):
    
    np_img = np.array(_image.raw_data)
    reshaped_img = np_img.reshape((IM_HEIGHT, IM_WIDTH, 4))
    cv2.putText(reshaped_img, f"Throttle     = {_truncate(control.throttle, 2)}",(430, 30), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)
    cv2.putText(reshaped_img, f"Steer        = {_truncate(control.steer, 2)}",(430, 60), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)
    cv2.putText(reshaped_img, f"Brake        = {_truncate(control.brake, 2)}",(430, 90), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)
    cv2.putText(reshaped_img, f"Speed(km/h) = {_truncate(speed, 2)}",(430, 120), cv2.FONT_HERSHEY_PLAIN, 1,(255,255,255),2)
    front_cam = reshaped_img[:, :, :3]
    global IMAGE_NO, IMG_PATH
    IMG_PATH = f"{FR_PATH}/{IMAGE_NO}.jpg"
 
    
    cv2.imwrite(IMG_PATH,front_cam)
    IMAGE_NO += 1
    return front_cam

def _truncate(number, decimals=0):
    """
    Returns a value truncated to a specific number of decimal places.
    """
    if not isinstance(decimals, int):
        raise TypeError("decimal places must be an integer.")
    elif decimals < 0:
        raise ValueError("decimal places has to be 0 or more.")
    elif decimals == 0:
        return math.trunc(number)

    factor = 10.0 ** decimals
    return math.trunc(number * factor) / factor

def _collision_data(event):
    # Eğer bir çarpışma gerçekleşirse çarpışmanın olduğu konum kaydedilir.
    print("COLLISION HAPPENED")
    event_loc = event.other_actor.get_location()
    return collision_hist.append((event_loc.x, event_loc.y)) 

def load_frames_from_folder():
    frames = []
    file_list = natsorted(os.listdir(FR_PATH))  
    for filename in file_list:
        print(filename)
        filepath = os.path.join(FR_PATH,filename)
        fr = cv2.imread(filepath)
        if fr is not None:
            frames.append((filepath, fr))
    return frames

def save_test_as_video(fps):
    # Video kayıtları için bir klasör oluşturulur
    try:
        os.makedirs(REC_PATH)
    except OSError:
        print ("Creation of the directory %s failed" % REC_PATH)
    else:
        print ("Successfully created the directory %s" % REC_PATH)

    print("Recording Test Drive Video...")
    
    frames = load_frames_from_folder()
    
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter("video.mp4",
                            fourcc,
                            fps, 
                            (IM_WIDTH, IM_HEIGHT))
    for frame in frames:
        # print(frame["File Path"])
        out.write(frame[1])
    out.release()
    
    print(f"Test drive video recorded")
    # for frame in frames:
    #     os.remove(frame["File Path"])

# Mevcut harita üzerindeki herhangi bir nesneyi işaretler.
def draw_string(world, object_location, color, string, life_time):
    world.debug.draw_string(object_location, string, draw_shadow=False,
                                       color=carla.Color(r=color[0], g=color[1], b=color[2]), life_time=life_time,
                                       persistent_lines=True)

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
