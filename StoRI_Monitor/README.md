# StoRI_Monitor

This library is the implementation of the StoRI and its monitor. **This StoRI implemenation assumes gaussian distributions at each point and linear atomic predicates**.

The `HyperplaneProbabilityFinder` function is used by the metric to find the probability of gaussian beliefs satisfying a linear predicate. The `internalStoRI` function is the actual implementation of both the StoRI and its monitor; one input to the function specifies whether the trace is complete or not, which dictates whether it finds the StoRI or its monitor. 

## Usage & Example
This implementation includes a parser for ease of use. The user must first define the atomic predicates, and then define the STL formula. The user can define STL subformulas to make this process easier. Consider the following specification that might apply to a robot:
$$\phi_3 = (\text{Puddle}\rightarrow(\lnot\text{Charger} U_{[0,3]}\text{Carpet}))U_{[0,10]}\text{Charger}$$
This, in plain english, reads *puddle means you must stay away from charger until drying off on the carpet within 3 minutes, and this must hold until reaching the charger within 10 minutes*. Here, Puddle, Charger, and Carpet are all conjunctions of linear predicates. Mathematically, thes predicates are captured by the inequality 
$$Ax > B,$$
where $A$ is a $1 \times n$ vector, $x$ is your $n \times 1$ state, and $B$ is some real scalar. 

#### Predicates
To stay general, this implementation does not assume a dimensionality of your state space, and so the user must define this. Consider the predicate: 
$$2x + 3y > 4$$
$$[2 \quad 3][x \quad y]^T > 4$$
To implement this, one would first begin by instantiating an instance of the monitor
>  StoRI_Monitor myMonitor; //Initialize monitor 

Then, the user creates a node and defines the A and B attributes:
> ASTNode* myNode = new ASTNode(5); //input "5" means predicate-type node \
myNode->A = Eigen::Vector2d(2,3);\
myNode->B = 4;\

Finally, **the user creates a map to label these predicate nodes with strings**, and adds this node to the map:
> std::map\<std::string,ASTNode*\> mymap;\
mymap["2x+3y>4"] = myNode;

This process can be cumbersome. Therefore, for the special case of *axis-aligned predicates* (i.e., $x>2$), we have a simpler implementation, thanks to the `axisAlignedPredicate` function:
>  StoRI_Monitor myMonitor; //Initialize monitor \
> std::map\<std::string,ASTNode*\> mymap;
>
> //"Traditional Implementation:"\
ASTNode* myNode = new ASTNode(5); \
myNode->A = Eigen::Vector2d(1,0); //values for x > 2\
myNode->B = 2;\
mymap["x>2"] = myNode;
>
> //EQUIVALENT implementation:\
  mymap["x>2"] = axisAlignedPredicate(2, 0, true, 2); 

where the inputs to `axisAlignedPredicate` in order are state dimension, index of state of interest, whether the operator is > or >= (as opposed to < or <= which would be "false"), and the value of B in the equations above

