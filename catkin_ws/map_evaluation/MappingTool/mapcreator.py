#! /usr/bin/env python
import xml.etree.ElementTree as ET
import os
import time
import signal
import subprocess
import numpy as np
import math
from collections import deque
import ntpath
import psutil

def killProcessTree(pid):
    try:
        parent = psutil.Process(pid)
    except psutil.NoSuchProcess:
        return False
    # children = parent.children(recursive=True)
    children = parent.children(recursive=False)
    for process in children:
        process.send_signal(signal.SIGTERM)
    return True

def getBridgeParams(bridge_node):
    params = ""
    paramnodes = bridge_node.findall('param')
    if paramnodes == None:
        return params
    for param in paramnodes:
        name = param.get('name')
        value = param.text
        if name!=None and value!=None:
            params = params+" "+name+":="+value
    if len(params)>0:
        params=params[1:]
    return params

def runAlgorithm(root,rsbag_command,alg_command,mapfile):
    print "Execute: ", alg_command
    time.sleep(10)
    alg = subprocess.Popen(alg_command, shell=True)
    alg_pid = alg.pid
    time.sleep(10)
    print "Execute: ", rsbag_command
    rsbag = subprocess.Popen(rsbag_command, shell=True)
    rsbag.communicate()
    time.sleep(5)
    saveMap(mapfile)
    time.sleep(15)
    print "Kill process: ", alg_pid
    killProcessTree(alg_pid)

def saveMap(filename):
    command = "rosrun map_server map_saver -f "+filename
    mapsaver = subprocess.Popen(command, shell=True)
    print "Execute: ", command
    time.sleep(7)
    if killProcessTree(mapsaver.pid):
        print "Successfully saved map!"
    else:
        print "Failed to save map!"

def forAllAlgorithms(root,rsbag_command,rsbag_name,rsbag_bridge_param):
    mapfilePath = root.find('map_path').text
    if mapfilePath==None:
        mapfilePath = "" #TODO: FIXME!!
    for alg in root.find('algorithms').findall('algorithm'):
        bridge_params = getBridgeParams(alg.find('bridge'))
        rsbrosbridge = "roslaunch "+root.find("rsb-ros_bridge").get('filepath')\
            +" "+rsbag_bridge_param+" "+bridge_params
        print "Execute: ", rsbrosbridge
        bridge = subprocess.Popen(rsbrosbridge, shell=True)
        bridge_pid = bridge.pid

        algFile = alg.find('file')
        algLaunch = os.path.join(algFile.text,algFile.get('name'))
        # Just one set of settings
        for single in alg.findall('single'):
            mapfile = mapfilePath+rsbag_name+"_"+alg.get('name')
            paramstring = ""
            for param in single.findall('param'):
                name = param.get('name')
                value = param.text
                if value!=None and name != None:
                    paramstring = paramstring+name+":="+value+" "
                    if param.get('mapF')!="0":
                        mapfile = mapfile+"_"+name+"_"+value
            command = "roslaunch "+algLaunch+" "+paramstring[:-1]
            runAlgorithm(root,rsbag_command,command,mapfile)

        # Cross evaluation of values for single parameters and groups of parameters with depending values
        for cross in alg.findall('cross'):
            crossparams = deque()
            crossmaps = deque()
            # single parameters
            for param in cross.findall('param'):
                name = param.get('name')
                if name != None:
                    paramdeq = deque()
                    mapdeq = deque()
                    for val in param.findall('value'):
                        value = val.text
                        if value !=None:
                            paramdeq.append(name+":="+value)
                            if param.get('mapF')!="0":
                                mapdeq.append(name+"_"+value)
                    if len(paramdeq)>0:
                        crossparams.append(paramdeq)
                        crossmaps.append(mapdeq)
            # group of parameters
            for group in cross.findall('group'):
                paramdeq = deque()
                mapdeq = deque()
                for setN in group.findall('set'):
                    paramstring = ""
                    mapstring = ""
                    for param in setN.findall('param'):
                        name = param.get('name')
                        value = param.text
                        if value != None and name != None:
                            paramstring = paramstring+name+":="+value+" "
                            if param.get('mapF')!="0":
                                mapstring = mapstring+"_"+name+"_"+value
                    if len(paramstring)>0:
                        paramdeq.append(paramstring[:-1])
                        mapdeq.append(mapstring)
                if len(paramdeq)>0:
                    crossparams.append(paramdeq)
                    crossmaps.append(mapdeq)
            if len(crossparams)<1:
                continue
            crossparamstrings = crossparams.pop()
            crossmapstrings = crossmaps.pop()
            while len(crossparams)>0:
                paramdeq = crossparams.pop()
                mapdeq = crossmaps.pop()
                c = len(crossparamstrings)
                for i in range(0,c):
                    oldstring = crossparamstrings.popleft()
                    oldmap = crossmapstrings.popleft()
                    for j in range(0,len(paramdeq)):
                        crossparamstrings.append(oldstring+" "+paramdeq[j])
                        crossmapstrings.append(oldmap+"_"+mapdeq[j])
            for i in range(0,len(crossparamstrings)):
                command = "roslaunch "+algLaunch+" "+crossparamstrings[i]
                mapfile = mapfilePath+rsbag_name+"_"+alg.get('name')+crossmapstrings[i]
                runAlgorithm(root,rsbag_command,command,mapfile)
        print "Kill process: ", bridge_pid
        killProcessTree(bridge_pid)


def forAllPlaybackSpeeds(root,rsbag_file,filePathName,name):
    rsbag_params = getBridgeParams(rsbag_file.find('bridge'))
    for speed in rsbag_file.findall('speed'):
        command = 'rsbagcl0.16 play -r"recorded-timing :speed '+speed.text+'" '+filePathName
        forAllAlgorithms(root,command,name,rsbag_params)

def forAllRsbagFiles(root):
    rsbag_files = root.find('rsbag_files').findall('rsbag_file')
    rsbag_path = root.find('rsbag_files').find('path').text
    for rsbag_file in rsbag_files:
        name = rsbag_file.get('name')
        fileName = rsbag_file.find('filename').text
        filePathName = os.path.join(rsbag_path,fileName)
        forAllPlaybackSpeeds(root,rsbag_file,filePathName,name)

tree = ET.parse('slam_mapping.xml')
root = tree.getroot()
forAllRsbagFiles(root)
