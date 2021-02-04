# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
from enum import Enum
import carla
from agents.navigation.agent import Agent, AgentState
from agents.navigation.local_planner import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.tools.misc import draw_string 

class BasicAgent(Agent):
    #  BasicAgent, belirli bir hedef noktasına ulaşmak için sahnelerde gezinen temel bir ajan uygular. 
    #  Bu ajan trafik ışıklarına ve diğer araçlara duyarlıdır.

    def __init__(self, vehicle, target_speed=20):
        super(BasicAgent, self).__init__(vehicle)

        self._proximity_tlight_threshold = 5.0  # meters
        self._proximity_vehicle_threshold = 10.0  # meters
        self._state = AgentState.NAVIGATING
        args_lateral_dict = {
            'K_P': 1,
            'K_D': 0.4,
            'K_I': 0,
            'dt': 1.0/20.0}
        self._local_planner = LocalPlanner(
            self._vehicle, opt_dict={'target_speed' : target_speed,
            'lateral_control_dict':args_lateral_dict})
        self._hop_resolution = 2.0
        self._path_seperation_hop = 2
        self._path_seperation_threshold = 0.5
        self._target_speed = target_speed
        self._grp = None

    def set_destination(self, location):
        # Bu fonksiyon, global yönlendirici tarafından döndürülen 
        # rotaya göre temsilcinin konumundan hedef konuma kadar bir yol noktası listesi oluşturur.
        

        start_waypoint = self._map.get_waypoint(self._vehicle.get_location())
        end_waypoint = self._map.get_waypoint(
            carla.Location(location.x, location.y, location.z))

        route_trace = self._trace_route(start_waypoint, end_waypoint)

        self._local_planner.set_global_plan(route_trace)

    def _trace_route(self, start_waypoint, end_waypoint):        
        # Bu fonksiyon global bir yönlendirici kurar ve 
        # başlangıç yol noktasından bitiş noktasına en uygun yolu döndürür
        

        # Global yönlendiricinin kurulması
        if self._grp is None:
            dao = GlobalRoutePlannerDAO(self._vehicle.get_world().get_map(), self._hop_resolution)
            grp = GlobalRoutePlanner(dao)
            grp.setup()
            self._grp = grp

        # Rota planı elde edilir
        route = self._grp.trace_route(
            start_waypoint.transform.location,
            end_waypoint.transform.location)

        return route

    def run_step(self, debug=False):
        # Navigasyonun bir adımını yürütün.

        # Aracın önünde bir engel mi var?
        hazard_detected = False

        # Güvenli navigasyon için ilgili unsurlar geri alınır, yani: trafik ışıkları ve diğer araçlar
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")
        
        # Olası engeller kontrol edilir
        blocking_vehicle_location = None
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))

            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True
            vehicle_location = vehicle.get_location()
            draw_string(self._world, vehicle.get_location(), (255, 0, 255) ,"BLOCKING VEHICLE", life_time=0.0)
            # Araç engellemesinin konumu kaydedilir
            blocking_vehicle_location = (vehicle_location.x, vehicle_location.y)

        # Trafik ışıklarının durumu kontrol edilir
        red_light_location = None
        light_state, traffic_light, object_location = self._is_light_red(lights_list)
        if light_state:
            if debug:
                print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))

            self._state = AgentState.BLOCKED_RED_LIGHT
            hazard_detected = True
            draw_string(self._world, object_location, (255, 255, 0), "RED LIGHT", life_time=0.0)
            # Kırmızı ışığın konumu kaydedilir
            red_light_location = (object_location.x, object_location.y)

        if hazard_detected:
            control = self.emergency_stop()
        else:
            self._state = AgentState.NAVIGATING
            # Standart yerel planlayıcı davranışı
            control = self._local_planner.run_step(debug=debug)
            # Otonom aracın durumu kontrol edilir
            if debug:
                print('NAVIGATING [{}]...)'.format(self._vehicle.id))
        draw_string(self._world, self._vehicle.get_location(), (0, 255, 255) ,"MY VEHICLE", life_time=0.0)
 
        return control, blocking_vehicle_location, red_light_location  

    # Otonom aracın durumu ekrana yazdırılır
    def show_road_option(self):
        target_road_option = self._local_planner.get_target_road_option()
        _control = self._vehicle.get_control()
        if int(_control.throttle) == 0 and int(_control.steer) == 0 and int(_control.brake) == 1:
            print("Road Option = STOP")
        else:
            if target_road_option == RoadOption.LEFT:
                print("Road Option = LEFT")
            elif target_road_option == RoadOption.RIGHT:
                print("Road Option = RIGHT")
            elif target_road_option == RoadOption.STRAIGHT:
                print("Road Option = STRAIGHT")
            elif target_road_option == RoadOption.LANEFOLLOW:
                print("Road Option = LANEFOLLOW")
            elif target_road_option == RoadOption.CHANGELANELEFT:
                print("Road Option = CHANGELANELEFT")
            elif target_road_option == RoadOption.CHANGELANERIGHT:
                print("Road Option = CHANGELANERIGHT")
        
    def done(self):
        # Aracın hedefine ulaşıp ulaşmadığı kontrol edilir.

        return self._local_planner.done()
