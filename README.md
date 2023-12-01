# Guided and Extended Hybrid A* 

Arxiv Pre-Print: https://arxiv.org/abs/2310.06974

## Disclaimer
This is work in progress and is prone to changes in the future, e.g. a shift to C++ only is planned. 

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
Build and install the planner...
```
./build.sh
./install.sh
```
...and run the sim.
```
./run_sim.sh
```

Per default, the visualization of the visited nodes and internal heuristics is enabled.
This slows down the visualization and can be disabled in ```visualization.py```

## Configuration
### Sim configuration
```src/freespace_planner/ros2/scripts/sim_config/sim_config.yml```  
The SCENARIO_NAME controls which map to use.
ALL_VISIBLE decides if the environment is previously known.

### Library configuration
```src/freespace_planner/library/config/config.yml```  
Here, all the library params can be adjusted. They can also be set in the Gui. Scroll down on the right side.

## Troubleshooting
### GLFW Error
If you receive the error: 
```
No protocol specified
Could not initialize GLFW!
```
Run the following in your console before running the docker
```
xhost +si:localuser:root
```

### Graphics Card not supported
If you receive the error
```
RuntimeError: OpenCV(4.8.0) /opt/opencv_contrib-4.8.0/modules/cudev/include/opencv2/cudev/grid/detail/copy.hpp:78: error: (-217:Gpu API call) no kernel image is available for execution on the device in function 'copy'
```
your graphics card is no longer supported. Then you must rebuild OpenCV from source.


## Contribute
You are welcome to contribute to every part of this project.
Given the fact, that the python part is already planned to be replaced by C++, it would be more beneficial to contribute to the C++ part :)

## Credits
The provided repository uses code which was published by other developers. A detailed enumeration can be found in the planner
under src/freespace_planner/LICENSES.yaml
- [fitpack](https://github.com/scipy/scipy/tree/main/scipy/interpolate/fitpack): spline routines of scipy
- [pyReedsShepp](https://github.com/ghliu/pyReedsShepp): RS Curve generation
- [VoronoiDiagramGenerator](https://web.archive.org/web/20131207065132/http://www.skynet.ie/~sos/mapviewer/voronoi.php): Voronoi diagram generation
- [nanoflann](https://github.com/jlblancoc/nanoflann): nearest neighbor determination

