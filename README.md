# Motion Planning for STL with the StoRI
The [Stochastic Robustness Interval](https://arxiv.org/abs/2210.04813) (StoRI) is a measure of how robustly a stochastic trajectory satisfies a Signal Temporal Logic (STL) specificiation. This repository contains implementation of the StoRI, its monitor, and implementation of its usage in a motion planning algorithm. 

---

## Code Overview 

The code includes a `CMakeLists.txt` for compilation purposes. The code is dependent on the [Open Motion Planning Library](http://ompl.kavrakilab.org) (OMPL) as well as [Eigen3](https://eigen.tuxfamily.org/index.php?title=Main_Page). The repository here contains source code for static libraries for a custom state space, the measure and monitor, and the motion planning algorithm. The repository also contains some MATLAB scripts for plotting and testing the planner/measure. 

The directories also contain their own READMEs with more information. Please refer to the

### Spaces

The state space implementation is a modified state space based on code from Qi Heng Ho. It allows us to describe a belief of a state in $\mathbb{R}^4$, allowing us to store the mean and covariance. 

### PrSTL_Monitor

This library is the implementation of the StoRI and its metric. There are 4 hardcoded Abstract Syntax Trees (ASTs) representing different STL specifications. **This implementation assumes that the formulas are defined with respect to linear predicates, and the system has linear dynamics**. The `HyperplaneProbabilityFinder` function is used by the metric to find the probability of gaussian beliefs satisfying a linear predicate. The `AriaMetric` function is the actual implementation of both the StoRI and its monitor; one input to the function specifies whether the trace is complete or not, which dictates whether it finds the StoRI or its monitor. 

Now is a good time to note that this code is very messy; there are many snippets that go unused or were attempts at creating a heuristic for the motion planning planning algorithm. The `isActive` and `CollectPredicates` functions are examples of such snippets. 

### stl_rrt_planner
This library contains implementation of a StoRI-RRT motion planning algorithm for a 4d linearized unicycle. I *believe* the implementation should be general, but that isn't tested and various bugs may pop up. This script is largely based off of the RRT implementation from OMPL, but with modifications for validity checking and solution checking using the StoRI and its monitor. One important thing to note is that there are many tunable hyperparameters and additional functions in the `stl_rrt.h` file. 

### main script
This is the source code which: 
- Can define a problem and solve it using the StoRI-RRT planner
- Acts as a meta planner to call the StoRI-RRT planner and asymptotically optimize for the StoRI of trajectories
- Can benchmark these planners
- Can process and evaluate hand-crafted trajectories from MATLAB

In the `main` function, there the user can choose to call either `planWithSimpleSetup` or `evaluateMatlabTrace`. The former is the motion planning algorithm, and takes input flags that define whether to optimize the trajectory and whether to benchmark the performance. Running the function creates csv files for the solution trajectory (including inputs), covariances, and the StoRI of the resulting trajectory. The latter evaluates hand-crafted trajectories created using MATLAB, used for testing and understanding the measure. 

### MATLAB Code 
This code is quite incomplete and will likely need to be modified to work properly. With the proper csv files in the right directory,
- Plot_Path_From_OMPL.m visualizes solutions found using the StoRI-RRT planner
- Metric_Testing_Makepath.m generates csvs of trajectories to be processed by `evaluateMatlabTrace` in the C++ main script
- Plot_Path_From_MATLAB.m plots these paths
- plot_tree.m plots the entire search tree from StoRI-RRT (even when incomplete) when the planner is configured correctly
- MonteCarloTesting.m simulates the found solution to gather a statistical success rate
    - This "success" is judged by the [Breach Toolbox](https://github.com/decyphir/breach/blob/master/README.md)
    - This script/toolbox uses the ".stl" files in this directory that describe the STL formulas. 
