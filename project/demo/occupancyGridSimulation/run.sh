#!/bin/bash

# Start the sending of random vectors every 125 ms
# RSB configurations are done by rsb.conf, which resides in this folder
./sendRandomIntVector/sendRandomIntVector -o /IR -a 12 -v 1000 &
# Receiving sensor data and bilding an occupancy grid map out of it
# RSB configurations are done explicitly by program options
./occupancyGridFromSocketProxBebot/occupancyGridFromSocketProxBebot -i /IR -o /maps/local/robot/ogm -s localhost -p 55555 \
                                                                    --obstacleStart 0.1 --mapResolution 0.0033214 --mapHeight 150 --mapWidth 150 &
# Print out the OGM
# RSB configurations are done by rsb.conf, which resides in this folder
./showOccupancyGrid/showOccupancyGrid -i /maps/local/robot/ogm &

