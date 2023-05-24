# Motion Planning for STL with the StoRI
The [Stochastic Robustness Interval](https://arxiv.org/abs/2210.04813) (StoRI) is a measure of how robustly a stochastic trajectory satisfies a Signal Temporal Logic (STL) specificiation. This repository contains implementation of the StoRI, its monitor, and implementation of its usage in a motion planning algorithm. 

---

## Code Overview 

The code includes a `CMakeLists.txt` for compilation purposes. The code is dependent on the [Open Motion Planning Library](http://ompl.kavrakilab.org) (OMPL) as well as [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page). The repository here contains source code for static libraries for a custom state space, the measure and monitor, and the motion planning algorithm. The repository also contains some MATLAB scripts for plotting and testing the planner/measure. 

The directories also contain their own READMEs with more information. Please refer to the StoRI_Monitor directory specifically as **the measure and monitor can be used as a standalone tool**. Furthermore, the branch "StoRI-light" provides only the implementation of the monitor and measure, without the motion planning algorithm. The branch "original-src" contains the original, unmodified source code used for the paper.


### Main Script
This is the source code which: 
- Can define a problem and solve it using the StoRI-RRT planner
- Acts as a meta planner to call the StoRI-RRT planner and asymptotically optimize for the StoRI of trajectories
- Can benchmark these planners

The motion planning algorithm takes input flags that define whether to optimize the trajectory and whether to benchmark the performance. Running the function creates csv files for the solution trajectory (including inputs), covariances, and the StoRI of the resulting trajectory. 

