#!/usr/bin/env python

import sys
import cv2
import numpy as np
import yaml

debug = 0
gui = 0
map_scaling = 0

do_wallthickness_interation = 1
iteration_start = 1
iteration_end = 40
iteration_increment = 1

def scale(value):
    global map_scaling
    global debug
    return (int)(value*map_scaling)

def parseConfigString(configstring):
    global map_scaling

    if(debug):
        print "\nWhole config-yaml string:\n" + str(configstring) + "\n"

    gt_free = configstring['gt_types']['free']
    gt_occupied = configstring['gt_types']['occupied']
    gt_unknown = configstring['gt_types']['unknown']

    offset_x = configstring['offset']['x']
    offset_y = configstring['offset']['y']

    map_x = configstring['map']['x']
    map_y = configstring['map']['y']
    map_scaling = configstring['map']['scaling']
    map_wallthickness = 0
    if(do_wallthickness_interation):
        map_wallthickness = iteration_start
    else:
        map_wallthickness = configstring['map']['wallthickness']

    while (True):
        image = np.zeros((scale(map_y+2*offset_y), scale(map_x+2*offset_x), 1), np.uint8)

        #creates unknown area
        cv2.rectangle(image, (scale(0), scale(0)), (image.shape[1], image.shape[0]), gt_unknown, cv2.FILLED)

        #create free area
        cv2.rectangle(image, (scale(offset_x), scale(offset_y)), (scale(map_x+offset_x), scale(map_y+offset_y)), gt_free, cv2.FILLED)

        # create map borders
        cv2.rectangle(image, (scale(offset_x), scale(offset_y)), (scale(map_x+offset_x), scale(map_y+offset_y)), gt_occupied, map_wallthickness)

        # draw every obstacle
        numb_obstacle = len(configstring['obstacle'])
        for i in range (0, numb_obstacle):
            try:
                if(debug):
                    print "Config from box" + str(i) + ":" + str(configstring['obstacle']['box'+str(i)])
                box_x = configstring['obstacle']['box'+str(i)]['x'] + offset_x
                box_y = configstring['obstacle']['box'+str(i)]['y'] + offset_y
                box_width = configstring['obstacle']['box'+str(i)]['width']
                box_height = configstring['obstacle']['box'+str(i)]['height']
                #creates unknown area in the box
                cv2.rectangle(image, (scale(box_x), scale(box_y)), (scale(box_x+box_width), scale(box_y+box_height)), gt_unknown, cv2.FILLED)
                #we need to build a wall for the box
                cv2.rectangle(image, (scale(box_x), scale(box_y)), (scale(box_x+box_width), scale(box_y+box_height)), gt_occupied, map_wallthickness)
                if (debug):
                    cv2.putText(image,'box'+str(i),(scale(box_x), scale(box_y)), cv2.FONT_HERSHEY_SIMPLEX, 1,128,2)
            except:
                print "In obstacle there is no element called: " + 'box'+str(i)
                continue

        if(gui):
            cv2.imshow("gt", image)
            cv2.waitKey(0)

        cv2.imwrite("gt_maps/gt_wallthickness_" + str(map_wallthickness) + ".png", image)
        if(not do_wallthickness_interation or map_wallthickness == iteration_end):
            return
        else:
            map_wallthickness = map_wallthickness + iteration_increment


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
