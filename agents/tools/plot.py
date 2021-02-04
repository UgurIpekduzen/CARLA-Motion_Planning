import matplotlib.pyplot as plt
            
class Plotter(object):
    def __init__(self, client, agent, origin, destination):
        self.episodes_dict = dict()
        self._client = client
        self._agent = agent
        self._origin = origin
        self._destination = destination
        self.map_name = None
        self._unique_rl_locs = []
        self._unique_bv_locs = [] 
        self._unique_col_locs = []
    
    def dict_append(self, dict_data_row):
        # Grafiklerin çizimi için simülasyondan toparlanan verileri bir sözlüğe ekler
        
        row = dict()
        row['Elapsed Seconds'] = dict_data_row[0]
        row['Speed'] = dict_data_row[1]
        row['Throttle'] = dict_data_row[2]
        row['Steer'] = dict_data_row[3]
        row['Brake'] = dict_data_row[4]
  
        i = len(self.episodes_dict)
        self.episodes_dict['ep_' + str(i + 1)] = row

        return 0

    def _dict_filter(self, key):
        # İstenen anahtar kelimeye göre sözlüğü filtreler

        col = []
        i = 0
        while i < len(self.episodes_dict):
            col.append(float(self.episodes_dict['ep_' + str(i + 1)][key]))
            i += 1
        return col

    def _unique_lists_append(self, rl_list, bv_list, col_list):
        # Simulasyondan toparlanan kırmızı ışık, engel araç ve çarpışma konumlarından eşsiz olanları seçer

        for el in rl_list:
            if el not in self._unique_rl_locs: 
                self._unique_rl_locs.append(el) 

        for el in bv_list:
            if el not in self._unique_bv_locs: 
                self._unique_bv_locs.append(el) 
        
        for el in col_list:
            if el not in self._unique_bv_locs: 
                self._unique_col_locs.append(el)

    def _draw_map_graph(self):
        # Haritanın grafiği çizilir
        
        print("Drawing Map Graph...")
        fig = plt.figure(1)
        fig.set_figheight(10)
        fig.set_figwidth(10)
        world = self._client.get_world()
        world_map = world.get_map()
        self.map_name = world_map.name
        
        waypoint_list = world_map.generate_waypoints(0.5)
        start_waypoint = world_map.get_waypoint(self._origin.location)
        end_waypoint = world_map.get_waypoint(self._destination.location)

        route = self._agent._trace_route(start_waypoint, end_waypoint)
        
        plt.title(f"Map of {self.map_name}")

        plt.plot(
            [wp.transform.location.x for wp in waypoint_list],
            [wp.transform.location.y for wp in waypoint_list],
            linestyle='', 
            markersize=5, 
            color='grey', 
            marker='x', 
            label="Waypoint"
        )

        plt.plot(
            [route_wp[0].transform.location.x for route_wp in route],
            [route_wp[0].transform.location.y for route_wp in route],
            linestyle='', 
            markersize=5, 
            color='blue', 
            marker='x', 
            label="Route"
        )
        
        plt.plot(
            self._origin.location.x,
            self._origin.location.y, 
            linestyle='', 
            markersize=10, 
            color='red', 
            marker='X', 
            label="Origin"
        )

        plt.plot(
            self._destination.location.x,
            self._destination.location.y, 
            linestyle='', 
            markersize=10, 
            color='green', 
            marker='X', 
            label="Destination"
        )

        plt.plot(
            [rl[0] for rl in self._unique_rl_locs],
            [rl[1] for rl in self._unique_rl_locs],
            linestyle='', 
            markersize=5, 
            color='yellow', 
            marker='X', 
            label="Red Light Stops on Route"
        )

        plt.plot(
            [bv[0] for bv in self._unique_bv_locs],
            [bv[1] for bv in self._unique_bv_locs],
            linestyle='', 
            markersize=5, 
            color='orange', 
            marker='X', 
            label="Blocking Vehicle Stops on Route"
        )

        plt.plot(
            [col[0] for col in self._unique_col_locs],
            [col[1] for col in self._unique_col_locs],
            linestyle='', 
            markersize=5, 
            color='magenta', 
            marker='X', 
            label="Collisions on Route"
        )

        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.05),
            fancybox=False, shadow=False, ncol=4)
    
    def _draw_speed_graph(self):
        # Hız grafiği çizilir

        print("Drawing Speed Graph...")
        fig = plt.figure(2)
        fig.set_figheight(10)
        fig.set_figwidth(10)
        speed_data = self._dict_filter('Speed')
        elapsed_secs = self._dict_filter('Elapsed Seconds')
        plt.title(f"Speed Graph of Actor {self._agent._vehicle.id}", fontsize=20)
        plt.xlabel("Elapsed Seconds(s)", fontsize=15)
        plt.ylabel("Speed(km/h)", fontsize=15)
        plt.plot(elapsed_secs, speed_data)
    
    def _draw_throttle_graph(self):
        # Gaz pedalı grafiği çizilir

        print("Drawing Throttle Graph...")
        fig = plt.figure(3)
        fig.set_figheight(10)
        fig.set_figwidth(10)
        throttle_data = self._dict_filter('Throttle')
        elapsed_secs = self._dict_filter('Elapsed Seconds')
        plt.title(f"Throttle Graph of Actor {self._agent._vehicle.id}", fontsize=20)
        plt.xlabel("Elapsed Seconds(s)", fontsize=15)
        plt.ylabel("Throttle", fontsize=15)
        plt.plot(elapsed_secs, throttle_data)

    def _draw_steer_graph(self):
        # Direksiyon yön grafiği çizilir    

        print("Drawing Steer Graph...")
        fig = plt.figure(4)
        fig.set_figheight(10)
        fig.set_figwidth(10)
        steer_data = self._dict_filter('Steer')
        elapsed_secs = self._dict_filter('Elapsed Seconds')
        plt.title(f"Steer Graph of Actor {self._agent._vehicle.id}", fontsize=20)
        plt.xlabel("Elapsed Seconds(s)", fontsize=15)
        plt.ylabel("Steer", fontsize=15)
        plt.plot(elapsed_secs, steer_data)    
    
    def _draw_brake_graph(self):
        # Fren grafiği çizilir 

        print("Drawing Brake Graph...")
        fig = plt.figure(5)
        fig.set_figheight(10)
        fig.set_figwidth(10)
        brake_data = self._dict_filter('Brake')
        elapsed_secs = self._dict_filter('Elapsed Seconds')
        plt.title(f"Brake Graph of Actor {self._agent._vehicle.id}", fontsize=20)
        plt.xlabel("Elapsed Seconds(s)", fontsize=15)
        plt.ylabel("Brake", fontsize=15)
        plt.plot(elapsed_secs, brake_data)
    
    def save_graphs(self):
        # Bütün grafikler kaydedilir

        self._draw_map_graph()
        plt.savefig(f"./graphs/{self.map_name}_map_graph.jpg", dpi=200)
        self._draw_speed_graph()
        plt.savefig(f"./graphs/{self.map_name}_speed_graph.jpg", dpi=200)
        self._draw_throttle_graph()
        plt.savefig(f"./graphs/{self.map_name}_throttle_graph.jpg", dpi=200)
        self._draw_steer_graph()
        plt.savefig(f"./graphs/{self.map_name}_steer_graph.jpg", dpi=200)
        self._draw_brake_graph()
        plt.savefig(f"./graphs/{self.map_name}_brake_graph.jpg", dpi=200)
        plt.show()
        print(f"All graphs of {self.map_name} saved")


 
