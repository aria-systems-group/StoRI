### MATLAB Code 
This code is quite incomplete and will likely need to be modified to work properly. With the proper csv files in the right directory,
- Plot_Path_From_OMPL.m visualizes solutions found using the StoRI-RRT planner
- Metric_Testing_Makepath.m generates csvs of trajectories to be processed by `evaluateMatlabTrace` in the C++ main script
- Plot_Path_From_MATLAB.m plots these paths
- plot_tree.m plots the entire search tree from StoRI-RRT (even when incomplete) when the planner is configured correctly
- MonteCarloTesting.m simulates the found solution to gather a statistical success rate
    - This "success" is judged by the [Breach Toolbox](https://github.com/decyphir/breach/blob/master/README.md)
    - This script/toolbox uses the ".stl" files in this directory that describe the STL formulas. 
