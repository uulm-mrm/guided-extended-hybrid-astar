# Guided and Extended Hybrid A* 
  * [Description](#description)
  * [Links](#links)
  * [Citation](#citation)
  * [Videos](#videos)
    + [Known and Unknown Environments](#known-and-unknown-environments)
    + [Advanced motion capabilities](#advanced-motion-capabilities)
  * [Setup](#setup)
  * [Configuration](#configuration)
    + [Sim configuration](#sim-configuration)
    + [Library configuration](#library-configuration)
  * [Troubleshooting](#troubleshooting)
    + [GLFW Error](#glfw-error)
  * [Contribute](#contribute)
  * [Credits](#credits)

## Description
A Hybrid-A* planner with early stopping for efficient path planning.  
Further, the capabilities of the U-Shift vehicles can be activated to enable the extended abilities to rotate around its rear axis. 

<img align="left" src="https://github.com/uulm-mrm/guided-extended-hybrid-astar/blob/main/path_with_tree_new_copyright.png" width="330">  
<img align="center" src="https://github.com/uulm-mrm/guided-extended-hybrid-astar/assets/57749046/df41c8cb-8261-4dd7-a64d-efae3bb90998" width="330">  


## Links
Arxiv Pre-Print: https://arxiv.org/abs/2310.06974  
IEE Explore: https://doi.org/10.1109/ITSC57777.2023.10422264

## Citation
The paper was published at the IEEE ITSC 2023 and can be cited as stated below.
```
@INPROCEEDINGS{10422264,
  author={Schumann, Oliver and Buchholz, Michael and Dietmayer, Klaus},
  booktitle={2023 IEEE 26th International Conference on Intelligent Transportation Systems (ITSC)}, 
  title={Efficient Path Planning in Large Unknown Environments with Switchable System Models for Automated Vehicles}, 
  year={2023},
  volume={},
  number={},
  pages={2466-2472},
  keywords={Adaptation models;Runtime;Navigation;Switches;Path planning;Planning;Standards},
  doi={10.1109/ITSC57777.2023.10422264}}

```

## Videos
### Known and Unknown Environments
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/k8ezypm78WQ/0.jpg)](https://www.youtube.com/watch?v=k8ezypm78WQ)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/hHTfiry8gd0/0.jpg)](https://www.youtube.com/watch?v=hHTfiry8gd0)

### Advanced motion capabilities
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5uQWnyPqYFw/0.jpg)](https://www.youtube.com/watch?v=5uQWnyPqYFw)

## Setup
Clone the repository and update the submodules.
```
git submodule update --init --recursive                                                      
```
Change into the docker directory and build the docker image. This may take a while.
```
cd docker
./build.sh
```
Run the docker, everything from here will be executed inside it.
```
./run.sh
```

Follow the instructions given at the start of the docker
```
colcon build  
source colcon_build/install/setup.bash
ros2 run freespace_planner simulation.py
```

## Configuration
### Sim configuration
```src/freespace_planner/ros2/scripts/sim_config/sim_config.yml```  
The SCENARIO_NAME controls which map to use.
ALL_VISIBLE decides if the environment is previously known.

### Library configuration
```
src/freespace_planner/library/config/config.yml  
src/freespace_planner/library/config/params.yml  
```  
Here, all the library params can be adjusted. Some can also be set during runtime in the Gui. 

## Troubleshooting
### GLFW Error
If you receive the error: 
```Could not initialize GLFW!```
Verify that the DISPLAY variable is set to the same value inside the docker as outside with ```echo $DISPLAY```and set it with e.g. ```export DISPLAY=:1``` 

## Contribute
You are welcome to contribute to every part of this project.
Given the fact, that the python part is already planned to be replaced by C++, it would be more beneficial to contribute to the C++ part :)

## Credits
The provided repository uses code which was published by other developers. A detailed enumeration can be found here: [src/freespace_planner/LICENSES.yaml](src/freespace_planner/LICENSES.yaml)
- [fitpack](https://github.com/scipy/scipy/tree/main/scipy/interpolate/fitpack): spline routines of scipy
- [pyReedsShepp](https://github.com/ghliu/pyReedsShepp): RS Curve generation
- [VoronoiDiagramGenerator](https://web.archive.org/web/20131207065132/http://www.skynet.ie/~sos/mapviewer/voronoi.php): Voronoi diagram generation

