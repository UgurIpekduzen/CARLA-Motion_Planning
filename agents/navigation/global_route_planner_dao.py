# Copyright (c) # Copyright (c) 2018-2020 CVC.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
import numpy as np


class GlobalRoutePlannerDAO(object):
    # Bu sınıf, GlobalRoutePlanner için carla sunucusu örneğinden 
    # veri almak için gerekli veri erişim katmanıdır.

    def __init__(self, wmap, sampling_resolution):
        self._sampling_resolution = sampling_resolution # Ara noktalar arasındaki örnekleme mesafesi
        self._wmap = wmap #carla.world nesnesi

    def get_topology(self):
        # Bu fonksiyon, sunucudan topolojiyi, yol noktası nesneleri çiftleri olarak 
        # yol segmentlerinin bir listesi olarak alır ve topolojiyi 
        # bir sözlük nesneleri listesi olarak işler.
    
        topology = []
        # Ayrıntılı bir topoloji oluşturmak için yol noktalarını alınır
        for segment in self._wmap.get_topology():
            wp1, wp2 = segment[0], segment[1]
            l1, l2 = wp1.transform.location, wp2.transform.location
            # Kayan nokta belirsizliğini önlemek için yuvarlama yapılır
            x1, y1, z1, x2, y2, z2 = np.round([l1.x, l1.y, l1.z, l2.x, l2.y, l2.z], 0)
            wp1.transform.location, wp2.transform.location = l1, l2
            seg_dict = dict()
            seg_dict['entry'], seg_dict['exit'] = wp1, wp2
            seg_dict['entryxyz'], seg_dict['exitxyz'] = (x1, y1, z1), (x2, y2, z2)
            seg_dict['path'] = []
            endloc = wp2.transform.location
            if wp1.transform.location.distance(endloc) > self._sampling_resolution:
                w = wp1.next(self._sampling_resolution)[0]
                while w.transform.location.distance(endloc) > self._sampling_resolution:
                    seg_dict['path'].append(w)
                    w = w.next(self._sampling_resolution)[0]
            else:
                seg_dict['path'].append(wp1.next(self._sampling_resolution)[0])
            topology.append(seg_dict)
        return topology

    def get_waypoint(self, location):
        # Bu fonksiyon, verilen konumda ara nokta döndürür
        
        waypoint = self._wmap.get_waypoint(location)
        return waypoint

    def get_resolution(self):
        # self._sampling_resolution için erişimci
        
        return self._sampling_resolution
