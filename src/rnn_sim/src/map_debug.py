#!/usr/bin/env python
#Standard Libraries
import numpy as np
import yaml
import time
import matplotlib.image as mpimg
import sys
import yaml
from PIL import Image
import math
#from scipy.linalg import block_diag

#Map Handling Functions
def load_map(filename):
    # im = mpimg.imread("/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/" + filename)
    # im_np = np.array(im)  #Whitespace is true, black is false
    #im_np = np.logical_not(im_np)    
    map_image = Image.open("/home/yuhan/catkin_ws/" + filename)
    return map_image

def load_map_yaml(filename):
    with open("/home/yuhan/catkin_ws/" + filename, "r") as stream:
            map_settings_dict = yaml.safe_load(stream)
    return map_settings_dict

#Node for building a graph
class Node:
    def __init__(self, point, parent_id, cost):
        self.point = point # A 3 by 1 vector [x, y, theta]
        self.parent_id = parent_id # The parent node id that leads to this node (There should only every be one parent in RRT)
        self.cost = cost # The cost to come to this node
        self.children_ids = [] # The children node ids of this node
        return

#Path Planner 
class PathPlanner:
    #A path planner capable of perfomring RRT and RRT*
    def __init__(self, map_filename, map_setings_filename):
        #Get map information
        self.occupancy_map = load_map(map_filename)
        self.map_shape = self.occupancy_map.size
        self.map_settings_dict = load_map_yaml(map_setings_filename)

        #Get the metric bounds of the map
        self.bounds = np.zeros([2,2]) #m
        self.bounds[0, 0] = self.map_settings_dict["origin"][0]
        self.bounds[1, 0] = self.map_settings_dict["origin"][1]
        self.bounds[0, 1] = self.map_settings_dict["origin"][0] + self.map_shape[0] * self.map_settings_dict["resolution"]
        self.bounds[1, 1] = self.map_settings_dict["origin"][1] + self.map_shape[1] * self.map_settings_dict["resolution"]
        print("map shape", self.map_shape)
        print("map bounds", self.bounds)
        # bounds = self.find_bounds(self.occupancy_map)
        # print(bounds)
        # cropped_image = self.occupancy_map.crop((bounds[0], bounds[2], bounds[1] + 1, bounds[3] + 1))
        # print("croped map shape", cropped_image.size)
        # cropped_image.save("crop.pgm")
        # map_data = self.map_settings_dict
        # map_data["image"] = "crop.pgm"
        # crop_yaml = "crop.yaml"
        # map_data["origin"] = self.computed_cropped_origin(self.occupancy_map, bounds, self.map_settings_dict["resolution"], self.map_settings_dict["origin"])
        # with open(crop_yaml, "w") as f:
        #     yaml.dump(map_data, f)
    def find_bounds(self, map_image):
        x_min = map_image.size[0]
        x_end = 0
        y_min = map_image.size[1]
        y_end = 0
        pix = map_image.load()
        for x in range(map_image.size[0]):
            for y in range(map_image.size[1]):
                val = pix[x, y]
                if val != 205:  # not unknown
                    x_min = min(x, x_min)
                    x_end = max(x, x_end)
                    y_min = min(y, y_min)
                    y_end = max(y, y_end)
        return 7, 1627, 7, 1627
    def computed_cropped_origin(self, map_image, bounds, resolution, origin):
        """ Compute the image for the cropped map when map_image is cropped by bounds and had origin before. """
        ox = origin[0]
        oy = origin[1]
        oth = origin[2]
 
        # First figure out the delta we have to translate from the lower left corner (which is the origin)
        # in the image system
        dx = bounds[0] * resolution
        dy = (map_image.size[1] - bounds[3]) * resolution
 
        # Next rotate this by the theta and add to the old origin
 
        new_ox = ox + dx * math.cos(oth) - dy * math.sin(oth)
        new_oy = oy + dx * math.sin(oth) + dy * math.cos(oth)
 
        return [new_ox, new_oy, oth]
    def computed_cropped_origin(self, map_image, bounds, resolution, origin):
        """ Compute the image for the cropped map when map_image is cropped by bounds and had origin before. """
        ox = origin[0]
        oy = origin[1]
        oth = origin[2]
 
        # First figure out the delta we have to translate from the lower left corner (which is the origin)
        # in the image system
        dx = bounds[0] * resolution
        dy = (map_image.size[1] - bounds[3]) * resolution
 
        # Next rotate this by the theta and add to the old origin
 
        new_ox = ox + dx * math.cos(oth) - dy * math.sin(oth)
        new_oy = oy + dx * math.sin(oth) + dy * math.cos(oth)
 
        return [new_ox, new_oy, oth]


def main():
    #Set map information
    map_filename1 = "jackal1_map.pgm"
    map_setings_filename1 = "jackal1_map.yaml"
    map_filename2 = "jackal1_proj_map.pgm"
    map_setings_filename2 = "jackal1_proj_map.yaml"
    map_filename3 = "jackal1_map_proj_map.pgm"
    map_setings_filename3 = "jackal1_map_proj_map.yaml"
    # map_filename = "mymap.pgm"
    # map_setings_filename = "mymap.yaml"

    #RRT precursor
    path_planner1 = PathPlanner(map_filename1, map_setings_filename1)
    path_planner2 = PathPlanner(map_filename2, map_setings_filename2)
    path_planner2 = PathPlanner(map_filename3, map_setings_filename3)
    # map_image = Image.open("/home/yuhan/catkin_ws/src/jackal/jackal_navigation_rtab/maps/" + map_filename)


if __name__ == '__main__':
    main()