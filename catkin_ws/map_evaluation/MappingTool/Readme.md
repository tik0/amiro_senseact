# MappingTool
This tool simplifies the generation of maps created by mapping and slam algorithms.
It requires, that each algorithm can be started from a ros launch file and expects,
that the sensor data is provided in a rsbag file and fed through a rsb-ros_bridge.

## Building
The tool is written in python 2.7 and can be executed without building.

## Dependencies
### The following python libraries are required:
- xml.etree.ElementTree as ET
- os
- time
- signal
- subprocess
- numpy as np
- math
- from collections: deque
- ntpath

### Further requirements:
- librsbspread0.16 or higher
  - (rsbagcl0.16 play)
- ros
  - roslaunch
- ros map_server package
  - rosrun map_server map_saver

## How to setup the xml file
### Structure:
```
root
├── algorithms
│   ├── algorithm
│   │   ├── bridge
│   │   │   ├── param
│   │   │   ...
│   │   │   └── param
│   │   ├── file
│   │   ├── single
│   │   │   ├── param
│   │   │   ...
│   │   │   └── param
│   │   ...
│   │   ├── single
│   │   ├── cross
│   │   │   ├── group
│   │   │   │   ├── set
│   │   │   │   │   ├── param
│   │   │   │   │   ...
│   │   │   │   │   └── param
│   │   │   │   └── set
│   │   │   ...
│   │   │   ├── group
│   │   │   ├── param
│   │   │   │   ├── value
│   │   │   │   ...
│   │   │   │   └── value
│   │   │   ...
│   │   │   └── param
│   │   ...
│   │   └── cross
│   ...
│   └── algorithm
├── rsbag_files
│   ├── rsbag_file
│   │   ├── bridge
│   │   │   ├── param
│   │   │   ...
│   │   │   └── param
│   │   ├── filename
│   │   ├── speed
│   │   ...
│   │   └── speed
│   ...
│   └── rsbag_file
└── rsb-ros_bridge
```
### Parameters and values:
The nodes 'root', 'algorithms'and  'rsbag_files' have no special attributes. Any provided attributes are ignored. To construct the final filename for the map it is possible to set a name attribute for the nodes 'algorithm', 'param' and 'rsbag_file'. If it is not necessary to log a certain parameter in the filename, set the 'param' attribute 'mapF' to "0". If this parameter is not set or set with a different value, then the name and value of the parameter is used within the filename for the map as long as you have specified a value for the parameter. Any parameter without a value is completely ignored.

To add rsbag-files specify their location in the 'path' node. For each rsbag-file create a 'rsbag_file' node and specify the filename in the 'filename' node.  Each rsbag-file can be played back with multiple speeds. For each speed add a new 'speed' node with the specific value. If a special configuration of the rsb to ros bridge is needed, it is possible to add parameters in an 'bridge' node. The 'name' attribute is the parameter name and the the value is specified in text of the 'param' node.

To add an algorithm, it is necessary to create a ros launch file for this algorithm which provides arguments to modify the algorithm. The filename is given with the name attribute of the 'file' node and the location in its text. It is also possible to configure the rsb to ros bridge with parameters in a 'bridge' node how it is done in the 'rsbag_file' node.

A single set of parameters can be given in a 'single' node but if the task is a cross-evaluation of different values for multiple parameters use a 'cross' node. Like each 'single' node represents one call of the algorithm, each node 'cross' represents one cross-evaluation. Within each 'cross' node specify the parameter in the name attribute of a 'param' node and each value to analyze in a 'value' node. To set depending parameters create a 'group' node. Each 'set' node in the 'group' is similar to a 'value' node for a parameter. In the 'set' node specify the parameter how it is done in a 'single' node. Each 'set' is then combined with each 'value' of the others parameters in the 'cross' node.

## Common problems
- exists an rsb.config in the same folder as the python script?
- exists a slam_mapping.xml in the same folder as the python script?
- is spread running?
- is the ros environment setup?
