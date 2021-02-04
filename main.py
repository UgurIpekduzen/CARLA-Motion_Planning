import carla
from agents.navigation.basic_agent import BasicAgent
import time
from agents.tools.plot import Plotter
from agents.tools.misc import draw_string, set_vehicle, get_speed, actor_list, collision_hist

def main():
    try:        
        # Bir Carla istemcisi oluşturma
        client = carla.Client('localhost', 2000)
        
        # Harita bilgileri
        world = client.get_world()
        
        # Harita üzerinde bir başlangıç ve varış noktası seçimi
        agent_origin = carla.Transform(carla.Location(x=229.97378540039062, y=67.59939575195312, z=0.44999998807907104),
                                        carla.Rotation(pitch=0.16792702674865723, yaw=91.39320373535156, roll=6.670134666819649e-09))
        agent_destination = carla.Transform(carla.Location(x=-4.364792346954346, y=-126.3371810913086, z=0.27530714869499207),
                                        carla.Rotation(pitch=0.0, yaw=91.41353607177734, roll=0.0))
        
        # Aracın ve onu kullanacak ajanın oluşturulması
        vehicle = set_vehicle(world, agent_origin)
        agent = BasicAgent(vehicle, target_speed=20)
        
        # Varış noktasına giden en kısa rotanın oluşturulması
        agent.set_destination(agent_destination.location)

        plots = Plotter(client, agent, agent_origin, agent_destination)

        prev_rl_locations = [] # Önceki kırmızı ışık konumları
        prev_bv_locations = [] # Önceki engel araç konumları
        prev_col_locations = [] # Önceki çarpışma konumları

        time.sleep(10)

        while True:
            if agent.done() != True:
                # başlangıç ve varış noktasının simülasyon üzerinde işaretlenmesi
                draw_string(world, agent_origin.location, (255, 0, 0) ,"ORIGIN", life_time=0.0)
                draw_string(world, agent_destination.location, (0, 255, 0) ,"DESTINATION", life_time=0.0)
                
                control, bv_loc, rl_loc = agent.run_step(debug=True)
                vehicle.apply_control(control)                
                
                # Grafik için veri toplama
                if rl_loc != None:
                    prev_rl_locations.append(rl_loc)

                if bv_loc != None:
                    prev_bv_locations.append(bv_loc)
                
                if len(collision_hist) != 0:
                    prev_col_locations.append(collision_hist[-1])

                snapshot = world.get_snapshot()
                
                plots.dict_append([snapshot.timestamp.elapsed_seconds, 
                                    get_speed(agent._vehicle),
                                    control.throttle,
                                    control.steer,
                                    control.brake ])

                agent.show_road_option()
            else:
                plots._unique_lists_append(prev_rl_locations, prev_bv_locations, prev_col_locations)
                plots.save_graphs()
                break   
    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

        for actor in actor_list:
                actor.destroy()
                print("All cleaned up!")

    for actor in actor_list:
                actor.destroy()
                print("All cleaned up!")

    
if __name__ == '__main__':
    main()