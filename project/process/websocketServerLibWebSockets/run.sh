#!/bin/bash
# Note: The -t flag enables the camera thread, which sends
#       the frames explicitly inprocess to the scope g_sOutScopeImage
#       To receive the frames, inprocess has to be enabled in rsb.conf
./websocketServerLibWebSockets --resource_path ./Server --port 7681 --thread_camera

