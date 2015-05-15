#!/bin/bash
scp mapBuilder/mapBuilder root@192.168.2.$1:~
scp localPlanner/localPlanner root@192.168.2.$1:~
scp frontierExploration/frontierExploration root@192.168.2.$1:~
#scp startExplo.sh root@192.168.2.$1:~

