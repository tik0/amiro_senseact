#/bin/bash

file=amiro_hokuyo_controller
# file=amiro_sick_controller
speed=1
infos=octomap_default
echo "file: ${file}"
echo "speed: ${speed}"
echo "infos: ${infos}"
rsbagcl0.16 play --start-index 950 -r"recorded-timing :speed ${speed}" ${file}.tide

# rosrun map_server map_saver -f ${file}_speed${speed}_${infos}
