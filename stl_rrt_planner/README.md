### stl_rrt_planner
This library contains implementation of a StoRI-RRT motion planning algorithm for a 4d linearized unicycle. I *believe* the implementation should be general, but that isn't tested and various bugs may pop up. This script is largely based off of the RRT implementation from OMPL, but with modifications for validity checking and solution checking using the StoRI and its monitor. One important thing to note is that there are many tunable hyperparameters and additional functions in the `stl_rrt.h` file. 

