#!/bin/bash

# Start the showing of images
# RSB configurations are done by rsb.conf, which resides in this folder
./showCamJpg/showCamJpg -i /frames &
# Start the sending of images
echo "Hit any key to close the camera device!"
./senseCamJpg/senseCamJpg -o /frames -d 0 -q 85