# StoRI_Monitor

This library is the implementation of the StoRI and its monitor. **This StoRI implemenation assumes gaussian distributions at each point and linear atomic predicates**.

The `HyperplaneProbabilityFinder` function is used by the metric to find the probability of gaussian beliefs satisfying a linear predicate. The `internalStoRI` function is the actual implementation of both the StoRI and its monitor; one input to the function specifies whether the trace is complete or not, which dictates whether it finds the StoRI or its monitor. 

## Usage & Example
This implementation includes a parser for ease of use. The user must first define the atomic predicates, and then define the STL formula. The user can define STL subformulas to make this process easier. Consider the following specification that might apply to a robot:
$$\phi_3 = (\text{Puddle}\rightarrow(\lnot\text{Charger} U_{[0,3]}\text{Carpet}))U_{[0,10]}\text{Charger}$$
This, in plain english, reads *puddle means you must stay away from charger until drying off on the carpet within 3 minutes, and this must hold until reaching the charger within 10 minutes*. Here, Puddle, Charger, and Carpet are all conjunctions of linear predicates. Mathematically, thes predicates are captured by the inequality 
$$Ax > B,$$
where $A$ is a $1 \times n$ vector, $x$ is your $n \times 1$ state, and $B$ is some real scalar. 

### Predicates
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

### Formulas
With predicates defined, the rest of the formula can be defined as a string
- **no disjunctions or implications**
  - $A \lor B = \lnot(\lnot A \land \lnot B)$
  - $A \rightarrow B = \lnot A \lor B = \lnot(A \land \lnot B)$
  - conjunction, $\land$, written as "&"
  - negation, $\lnot$, written as "!"
  - until, $U$, written as "U"
  - eventually, $\Diamond$, written as "F"
  - globally, $\square$, written as "G"
- No unbounded formulae
  - Temporal operators must be followed by an interval bounded by brackets
    - i.e., "F[0,2](A\)", "(A)U[4,5](B\)"
- Must be diligent w/ parenthesis
  - everything, include the predicates, should have parenthesis around them. When in doubt, throw another set on!
  - ex, don't write `!predicate1`, but `(!(predicate1))`
- **One operator per parentheses**
  - ie, not "(A)&(B)&(C\)", but "((A)&(B))&C"
  - tricky and cumbersome, but due to binary tree interpretation of syntax tree

Recall the formula from above:

$$\phi_3 = (\text{Puddle}\rightarrow(\lnot\text{Charger} U_{[0,3]}\text{Carpet}))U_{[0,10]}\text{Charger}$$

With predicates defined, the user can write out the entire formula based on predicates, or alternatively choose to first define subformulae.

For example, it's often helpful to first define regions (the puddle, charger, and carpet from above):
> //build regions\
> ASTNode* puddle = myMonitor.BuildAST("((y>2)&(y<3))&(x<2.5)",mymap);\
> mymap["puddle"] = puddle;\
>
> ASTNode* charger = myMonitor.BuildAST("(x>2)&(y>4)",mymap);
> mymap["charger"] = charger;
>
> ASTNode* carpet = myMonitor.BuildAST("(x<1)&(y>4)",mymap);
> mymap["carpet"] = carpet;

Next, the user might choose to define subformulas first. For example, the portion $\lnot \text{Charger}U_{[0,3]}\text{Carpet}$ might be defined first as ad "dry off first" specification:
> ASTNode* dryOffFirst = myMonitor.BuildAST("(!(charger))U[0,3](carpet)",mymap);
> mymap["dryOffFirst"] = dryOffFirst;

Then, the portion $\text{Puddle}\rightarrow(\lnot \text{Charger}U_{[0,3]}\text{Carpet})$ can instead be written as $\text{Puddle}\rightarrow \text{dryOffFirst}$ as a "safety" condition:

> ASTNode* safety = myMonitor.BuildAST("!((puddle)&(!(dryOffFirst)))",mymap);
> mymap["safety"] = safety;

Finally, this lets the user write the formula as $\text{safety}U_{[0,10]}\text{Charger}$:
> myMonitor.myRootNode = myMonitor.BuildAST("((workspace)&(safety))U[0,10](charger)",mymap);

Notice above that the actual predicate is set to the attribute of the monitor `myRootNode`. This is the node the monitor will look at when evaluating the StoRI

This formula is provided in the source code using function `myMonitor.BuildForm3()`

### Evaluating the StoRI
Once the formula is defined, the StoRI or StoRI Monitor can be evaluated by calling:
> myMonitor.AriaMetric(timeSignal, stateSignal, covSignal, intervalContainer, isTraceComplete);

Here, 
- timeSignal is a $m\times 1$ std::vector<double> of the real time values of the data
- stateSignal is a $m\times n$ std::vector<Eigen::MatrixXd> of the state values of the data
- covSignal is a $m\times (n \times n)$ std::vector<Eigen::MatrixXd> of the covariance values of the data (again, implementation assumes gaussian uncertainty)
- intervalContainer is a pointer to a 2 element array that the function will use to store the value of the interval
- isTracePartial is a boolean that
  - if true, will tell the function to evaluate the StoRI
  - if false, will tell the function to evaluate the StoRI Monitor
