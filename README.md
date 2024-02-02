# Guided and Extended Hybrid A* 
A Hybrid-A* planner with early stopping for efficient path planning. Further, the capabilities of the U-Shift vehicles can be activated to enable the extended abilities to rotate around its rear axis. 

<img align="center" src="https://github.com/uulm-mrm/guided-extended-hybrid-astar/blob/main/path_on_parking_search_tree.png" width="400">

## Videos
### Known and Unknown Environments
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/k8ezypm78WQ/0.jpg)](https://www.youtube.com/watch?v=k8ezypm78WQ)
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/hHTfiry8gd0/0.jpg)](https://www.youtube.com/watch?v=hHTfiry8gd0)

### Advanced motion capabilities
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/5uQWnyPqYFw/0.jpg)](https://www.youtube.com/watch?v=5uQWnyPqYFw)

## Citation
Arxiv Pre-Print: https://arxiv.org/abs/2310.06974  
The paper can be cited as stated below.
```
@article{schumann2023efficient,
   author={Oliver Schumann and Michael Buchholz and Klaus Dietmayer},
   journal = {preprint, accepted to IEEE Int. Conf. on Intell. Transportation Syst. (ITSC)},
   title={Efficient Path Planning in Large Unknown Environments with Switchable System Models for Automated Vehicles}, 
   year = {2023},
}
```

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
The provided repository uses code which was published by other developers. A detailed enumeration can be found here: [src/freespace_planner/LICENSES.yaml](src/freespace_planner/LICENSES.yaml)
- [fitpack](https://github.com/scipy/scipy/tree/main/scipy/interpolate/fitpack): spline routines of scipy
- [pyReedsShepp](https://github.com/ghliu/pyReedsShepp): RS Curve generation
- [VoronoiDiagramGenerator](https://web.archive.org/web/20131207065132/http://www.skynet.ie/~sos/mapviewer/voronoi.php): Voronoi diagram generation
- [nanoflann](https://github.com/jlblancoc/nanoflann): nearest neighbor determination

