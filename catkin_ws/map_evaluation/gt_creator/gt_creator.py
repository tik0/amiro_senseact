#!/usr/bin/env python

import sys
import cv2
import numpy as np
import yaml

########################
# Adjustable Parameter #
########################
debug = 1
print_box_name = 0
gui = 0

#############
# Variables #
#############
# wt = wallthickness
wt_iteration_start = 1
wt_iteration_end = 40

map_scaling = 0

def scale(value):
    global map_scaling
    return (int)(value*map_scaling)


def parseConfigString(configstring):
    global map_scaling

    if(debug):
        print "\nWhole config-yaml string:\n" + str(configstring) + "\n"

    gt_free = configstring['gt_types']['free']
    gt_occupied = configstring['gt_types']['occupied']
    gt_unknown = configstring['gt_types']['unknown']

    m_per_pixel = configstring['cam']['m_per_pixel']

    offset_x = configstring['offset']['x']
    offset_y = configstring['offset']['y']

    map_x = configstring['map']['x']
    map_y = configstring['map']['y']

    resolutions = configstring['gt_resolutions']

    for res in resolutions:
        map_scaling = m_per_pixel/res*3;
        for wt in range (wt_iteration_start, wt_iteration_end):
            print "start interation with map_wallthickness: " + str(wt)
            image = np.zeros((scale(map_y+2*offset_y), scale(map_x+2*offset_x), 1), np.uint8)

            #creates unknown area
            cv2.rectangle(image, (scale(0), scale(0)), (image.shape[1], image.shape[0]), gt_unknown, cv2.FILLED)

            #create free area
            cv2.rectangle(image, (scale(offset_x), scale(offset_y)), (scale(map_x+offset_x), scale(map_y+offset_y)), gt_free, cv2.FILLED)

            # create map borders
            cv2.rectangle(image, (scale(offset_x), scale(offset_y)), (scale(map_x+offset_x), scale(map_y+offset_y)), gt_occupied, scale(wt))

            # draw every obstacle
            numb_obstacle = len(configstring['obstacle'])
            for i in range (0, numb_obstacle):
                try:
                    if(debug):
                        print "draw box" + str(i) + ":" + str(configstring['obstacle']['box'+str(i)])
                    box_x = configstring['obstacle']['box'+str(i)]['x'] + offset_x
                    box_y = configstring['obstacle']['box'+str(i)]['y'] + offset_y
                    box_width = configstring['obstacle']['box'+str(i)]['width']
                    box_height = configstring['obstacle']['box'+str(i)]['height']
                    #creates unknown area in the box
                    cv2.rectangle(image, (scale(box_x), scale(box_y)), (scale(box_x+box_width), scale(box_y+box_height)), gt_unknown, cv2.FILLED)
                    #we need to build a wall for the box
                    cv2.rectangle(image, (scale(box_x), scale(box_y)), (scale(box_x+box_width), scale(box_y+box_height)), gt_occupied, scale(wt))
                    if (print_box_name):
                        cv2.putText(image,'box'+str(i),(scale(box_x), scale(box_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,128,2)
                except:
                    print "In obstacle there is no element called: " + 'box'+str(i)
                    continue

            if(gui):
                cv2.imshow("gt", image)
                cv2.waitKey(0)

            cv2.imwrite("gt_maps/gt_wallthickness_" + str(wt) + "_resolution_" + str(res) + ".png", image)
            print " "

# MAIN
if __name__ == "__main__":

    # Read sysargs
    config = "config.yaml"
    if len(sys.argv) > 1:
        config = str(sys.argv[1])
    print "Use configfile: " + config

    # Read configfile
    with open(config, 'r') as stream:
        try:
            parseConfigString(yaml.load(stream))
        except yaml.YAMLError as exc:
            print(exc)
