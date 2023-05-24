#include <iostream>
#include <ostream>
#include <vector>
#include "PrSTL_Monitor.h"
#include <map>
#include <cassert>

//TODO - something is off by a timestep or two, somewhere (look at online monitoring examples)
//TODO - replace "buildform" functions by simply doing it live in main
//TODO - in StoRI calculations, add check if a node has children or not and add errors where helpful

//long term todos:
// - make parser
// - make branch - identifier 
// - make reach-conjunction-tree identifier 


void PrSTL_Monitor::BuildForm1()
{
  // initialize predicate dict
  std::map<std::string,
  ASTNode*> mymap;

  mymap["x>3"] = axisAlignedPredicate(4, 0, true, 3);
  mymap["x<4"] = axisAlignedPredicate(4, 0, false, 4);
  mymap["y>2"] = axisAlignedPredicate(4, 2, true, 2);
  mymap["y<3"] = axisAlignedPredicate(4, 2, false, 3);
  mymap["x>1"] = axisAlignedPredicate(4, 0, true, 1);
  mymap["x<2"] = axisAlignedPredicate(4, 0, false, 2);
  mymap["x>0"] = axisAlignedPredicate(4, 0, true, 0);
  mymap["y>0"] = axisAlignedPredicate(4, 2, true, 0);

  //build regions
  ASTNode* goal = this->BuildAST("((x>3)&(x<4))&((y>2)&(y<3))",mymap);
  mymap["goal"] = goal;

  ASTNode* obstacle = this->BuildAST("((x>1)&(x<2))&((y>2)&(y<3))",mymap);
  mymap["obstacle"] = obstacle;

  ASTNode* workspace = this->BuildAST("((x>0)&(x<4))&((y>0)&(y<3))",mymap);
  mymap["workspace"] = workspace;

  //build specification
  this->myRootNode = this->BuildAST("((!(obstacle))&(workspace))U[4,6](goal)",mymap);

  //optionaly (goofy) heuristic info
  heuristicInfo.insert(std::pair<int,std::vector<int>>(1,{2}));
  simpleHeuristicInfo.push_back({3.5,2.5});
}

void PrSTL_Monitor::BuildForm2()
{
  // initialize predicate dict
  std::map<std::string,
  ASTNode*> mymap;

  // build predicates
  mymap["x>0"] = axisAlignedPredicate(4, 0, true, 0);
  mymap["x<4"] = axisAlignedPredicate(4, 0, false, 4);
  mymap["y>-2"] = axisAlignedPredicate(4, 2, true, -2);
  mymap["y<2"] = axisAlignedPredicate(4, 2, false, 2);
  mymap["x>1.5"] = axisAlignedPredicate(4, 0, true, 1.5);
  mymap["x<3.5"] = axisAlignedPredicate(4, 0, false, 3.5);
  mymap["y>-0.5"] = axisAlignedPredicate(4, 2, true, -0.5);
  mymap["y<0.5"] = axisAlignedPredicate(4, 2, false, 0.5);
  mymap["x>2"] = axisAlignedPredicate(4, 0, true, 2);
  mymap["x<3"] = axisAlignedPredicate(4, 0, false, 3);
  mymap["y>1"] = axisAlignedPredicate(4, 2, true, 1);
  mymap["y<-1"] = axisAlignedPredicate(4, 2, false, -1);

  //build regions
  ASTNode* goal1 = this->BuildAST("((x>2)&(x<3))&(y>1)",mymap);
  mymap["goal1"] = goal1;

  ASTNode* goal2 = this->BuildAST("((x>2)&(x<3))&(y<-1)",mymap);
  mymap["goal2"] = goal2;

  ASTNode* obstacle = this->BuildAST("((x>1.5)&(x<3.5))&((y>-0.5)&(y<0.5))",mymap);
  mymap["obstacle"] = obstacle;

  ASTNode* workspace = this->BuildAST("((x>0)&(x<4))&((y>-2)&(y<2))",mymap);
  mymap["workspace"] = workspace;

  //build specification
  this->myRootNode = this->BuildAST("(((workspace)&(!(obstacle)))U[0,10](goal1))&(((workspace)&(!(obstacle)))U[0,10](goal2))",mymap);

  //optionaly (goofy) heuristic info
  heuristicInfo.insert(std::pair<int,std::vector<int>>(2,{4}));
  heuristicInfo.insert(std::pair<int,std::vector<int>>(3,{11}));

  simpleHeuristicInfo.push_back({2.5,1.5});
  simpleHeuristicInfo.push_back({2.5,-1.5});
}

void PrSTL_Monitor::BuildForm3()
{
  // initialize predicate dict
  std::map<std::string,
  ASTNode*> mymap;

  // build predicates
  mymap["x>0"] = axisAlignedPredicate(4, 0, true, 0);
  mymap["x<3"] = axisAlignedPredicate(4, 0, false, 3);
  mymap["y>0"] = axisAlignedPredicate(4, 2, true, 0);
  mymap["y<5"] = axisAlignedPredicate(4, 2, false, 5);
  mymap["y>2"] = axisAlignedPredicate(4, 2, true, 2);
  mymap["y<3"] = axisAlignedPredicate(4, 2, false, 3);
  mymap["x<2.5"] = axisAlignedPredicate(4, 0, false, 2.5);
  mymap["y>4"] = axisAlignedPredicate(4, 2, true, 4);
  mymap["x<1"] = axisAlignedPredicate(4, 0, false, 1);
  mymap["x>2"] = axisAlignedPredicate(4, 0, true, 2);

  //build regions
  ASTNode* puddle = this->BuildAST("((y>2)&(y<3))&(x<2.5)",mymap);
  mymap["puddle"] = puddle;

  ASTNode* charger = this->BuildAST("(x>2)&(y>4)",mymap);
  mymap["charger"] = charger;

  ASTNode* carpet = this->BuildAST("(x<1)&(y>4)",mymap);
  mymap["carpet"] = carpet;

  ASTNode* workspace = this->BuildAST("((x>0)&(x<3))&((y>0)&(y<5))",mymap);
  mymap["workspace"] = workspace;

  //build subspecifications
    //Puddle -> (!charge U carpet)
    // = !Puddle OR (!charge U carpet) 
    // = !(Puddle AND !(!charge U carpet))
  ASTNode* dryOffFirst = this->BuildAST("(!(charger))U[0,3](carpet)",mymap);
  mymap["dryOffFirst"] = dryOffFirst;

  ASTNode* safety = this->BuildAST("!((puddle)&(!(dryOffFirst)))",mymap);
  mymap["safety"] = safety;

  //build specification
  this->myRootNode = this->BuildAST("((workspace)&(safety))U[0,10](charger)",mymap);

  //optionaly (goofy) heuristic info
  simpleHeuristicInfo.push_back({2.5,4.5});
  simpleHeuristicInfo.push_back({0.5,4.5});
}

ASTNode* PrSTL_Monitor::axisAlignedPredicate(int stateDim, int index, bool geq, double Bval)
{
  //stateDim = dimension of state
  //index = index (zero-based) of state of interest
  //geq = true if "greater than or equal to", false if "less than or equal to"
  //Bval = value state mus be less than or equal to
  ASTNode* myNode = new ASTNode(5);
  myNode->A = Eigen::VectorXd::Zero(stateDim);
  if (geq) {
    myNode->A(index) = 1;
    myNode->B = Bval;
  }else {
    myNode->A(index) = -1;
    myNode->B = -1*Bval;
  }
  return myNode;
}

ASTNode* PrSTL_Monitor::BuildAST(std::string strFormula, std::map<std::string,ASTNode*> predicates)
{
  // TODO
    // - create predicate class
      // Define "atomic predicates" as an inherited class of ASTNode w/ two arguments, A and b. Have them create a map/dictionary from the string to the predicate
    // - make the AST smarter w/ BST THIS IS, LIKE, A CRUCIAL FIRST STEP
    // make parser resilient to spaces 
    // make parser return an error for "non-binarized" data
  
  // REQUIREMENTS/RESTRINCTIONS
  // - Gaussianity, of course
  // - Linear predicates, of course
  // - Must define predicates before formula
  // - Must be diligent w/ parenthesis
    // can TRY to account for this via errors
  // - Must "write with binary tree interpretation in mind"
    // can TRY to account for this via errors
  // - No unbounded formulae
    // can give error if no time bounds given
  // - No disjunctions or implications (conjunctions and negations!)
    // can TRY to account for this via errors
  // - Must follow standard for temporal operators and their time bounds

  // if it's a predicate
  if (predicates.count(strFormula)>0) 
  {
    return predicates[strFormula]; 
  } 

  // if it's negation
  else if (strFormula[0] == '!') 
  {
    // create new negation node
    ASTNode* myNode = new ASTNode(2);  

    //parse remaining tree as child node
    myNode->left = BuildAST(strFormula.substr(1), predicates); 

    //return the node
    return myNode;
  }

  // if it's eventually or globally
  else if (strFormula[0] == 'F' || strFormula[0] == 'G') 
  {
    // create node
    ASTNode* myNode;
    if (strFormula[0] == 'F')
    {
      myNode = new ASTNode(7);
    }else {
      myNode = new ASTNode(6);
    }

    //Identify Time Bounds
    assert(("Temporal Operators must be followed by bracketed time bounds, Ex: F[0,2]",strFormula[1] == '[')); 
    int charIndex = strFormula.find(','); //find comma
    assert(("Temporal Operators cannot be unbounded, must separate time bounds with a comma, Ex: F[0,2]",charIndex != std::string::npos));
    std::string leftString = strFormula.substr(2,(charIndex)); //split string at comma
    std::string rightString = strFormula.substr(charIndex+1);
    myNode->leftbound = std::stod(leftString); //save left temporal bound
    charIndex = rightString.find(']'); //find right bracket
    assert(("Must close interval with a right bracket, Ex: F[0,2]",charIndex != std::string::npos));
    leftString = rightString.substr(0,(charIndex)); //split string at bracket
    rightString = rightString.substr(charIndex+1); 
    myNode->rightbound = std::stod(leftString); //save right temporal bound
                                                //
    //parse remaining tree as child node
    myNode->left = BuildAST(rightString, predicates); 

    //return the node
    return myNode;
  }

  // if it's a parenthesis oof
  else if (strFormula[0] == '(')
  {
    //iterate through string, looking for a "parenthetical chunk"
    int myCounter = 0;
    int index = 0;
    do
    {
      bool test = (index < (strFormula.length()-1));
      assert(("Parenthetical mismatch, check parenthesis",index <= (strFormula.length()-1))); //if we reach the end of the string (and index is not zero)
      if (strFormula.at(index)=='(') 
      {
        myCounter--;  
      }
      else if (strFormula.at(index)==')') {
        myCounter++;
      }
      index++;
    } while(myCounter != 0);

    // if string is empty, remove parenthesis from front and back and feed it to the function again
    if (index == strFormula.length())
    {
      return BuildAST(strFormula.substr(1,(strFormula.length()-2)), predicates); 
    } 
    
    // otherwisre, split into left and right side of symbol using index we kept
    std::string leftString = strFormula.substr(0,index);
    std::string rightString = strFormula.substr(index+1);
    
    // ensure that the operator is not disjuction
    assert(("Disjunction is no good, please use conjucntion. i.e., A|B = !(!A&!B)",strFormula.at(index)!='|')); // check for disjunction

    // if operator is conjunction
    if (strFormula.at(index)=='&') 
    {
      ASTNode* myNode = new ASTNode(1);  
      myNode->left = BuildAST(leftString, predicates); 
      myNode->right = BuildAST(rightString, predicates); 

      //return the node
      return myNode;
    } 

    // else if operator is until
    else if (strFormula.at(index)=='U') 
    {
      
      // create the node, find the left child info
      ASTNode* myNode = new ASTNode(3);  

      myNode->left = BuildAST(leftString, predicates); 

      strFormula = rightString;

      //identify time bounds
      assert(("temporal operators must be followed by bracketed time bounds, ex: f[0,2]",strFormula[0] == '[')); 
      int charindex = strFormula.find(','); //find comma
      assert(("temporal operators cannot be unbounded, must separate time bounds with a comma, ex: f[0,2]",charindex != std::string::npos));
      std::string leftstring = strFormula.substr(1,(charindex)); //split string at comma
      std::string rightstring = strFormula.substr(charindex+1);
      myNode->leftbound = std::stod(leftstring); //save left temporal bound
      charindex = rightstring.find(']'); //find right bracket
      assert(("must close interval with a right bracket, ex: f[0,2]",charindex != std::string::npos));
      leftstring = rightstring.substr(0,(charindex)); //split string at bracket
      rightstring = rightstring.substr(charindex+1); 
      myNode->rightbound = std::stod(leftstring); //save right temporal bound

      //parse remaining tree as child node
      myNode->right = BuildAST(rightstring, predicates); 

      //return the node
      return myNode;
    }
  } 

  // else, assert general error
  else 
  {
    assert(("error, something went wrong, say something about link to docs or tips" == 0));
  }
}

bool PrSTL_Monitor::HyperplaneCCValidityChecker(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const 
{
    //Adapted from Eric Pairet (Blackmore)
    //IN:
    //      A and B represent Linear Hyperplane (want Ax > B or Ax-b > 0 to "be valid")
    //      X is the mean of the gaussion random vector of interest 
    //      PX is the covariance of the gaussion random vector of interest
	
    bool valid = false;
    double delta = 0.2; //make this an input later
	double muV, PV, c;
	Eigen::MatrixXd Pv_2, muV_2;

    //Find mean and covariance of univariate gaussian V 
    muV_2 = A.transpose()*X;
    muV = muV_2(0, 0) - B; //note to self: do I have to split this into steps? matrix vs double?
    Pv_2 = A.transpose() * PX * A; //covariance of V
	PV = sqrt(Pv_2(0, 0));

    //Calculate c
    c = sqrt(2) * PV* boost::math::erf_inv(1 - (2*delta)); 

    //Check if constraint is satisfied
    if (muV > c) {
    valid = true;
    }
	return valid;
}

double PrSTL_Monitor::HyperplaneProbabilityFinder(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const 
{
    //IN:
    //      A and B represent Linear Hyperplane (want Ax < B or Ax-b < 0 to "be valid")
    //      X is the mean of the gaussion random vector of interest 
    //      PX is the covariance of the gaussion random vector of interest
    //OUT:
    //      Probability of satisfaction
	
    bool valid = false;
	double muV, PV, c, delta;
	Eigen::MatrixXd Pv_2, muV_2;

    //Find mean and covariance of univariate gaussian V 
    muV_2 = A.transpose()*X;
    muV = muV_2(0, 0) - B; //note to self: do I have to split this into steps? matrix vs double?
    Pv_2 = A.transpose() * PX * A; //covariance of V
	PV = sqrt(Pv_2(0, 0));

    //Calculate c (different c than above, this is just a middle step to make computations easier)
    c = muV/(sqrt(2) * PV);
    
    delta = (1 - boost::math::erf(c))/2; 

    //Check if constraint is satisfied
    return (1 - delta); //P(satisfaction) = 1 - P(violation) 
}

void PrSTL_Monitor::internalStoRI(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, ASTNode* myNode, bool CompleteTrace)
{
    if (myNode->nodetype==4) { //"True" Node
        Interval[0]= 1;
        Interval[1] = 1;
    }
    if (myNode->nodetype==5) { //Predicate Node
        double prob = this->HyperplaneProbabilityFinder(myNode->A,myNode->B, (*meanTrace)[0], (*covtrace)[0]);
        Interval[0] = prob;
        Interval[1] = prob;
    }
    if (myNode->nodetype==1) { //Conjunction
        // create intervals for the children
        double intervalL[2];
        double intervalR[2];

        // calculate intervals for the children
        this->internalStoRI(timevec,meanTrace,covtrace,intervalL,myNode->left, CompleteTrace); 
        this->internalStoRI(timevec,meanTrace,covtrace,intervalR,myNode->right, CompleteTrace); 

        //Apply Rules
        Interval[0] = intervalL[0] + intervalR[0] - 1; //lower bound

        if (Interval[0] < 0) { // account for negative probability
        Interval[0] = 0;
        }

        if (intervalL[1] < intervalR[1]) { //upperbound is left upper bound if it's smaller
        Interval[1] = intervalL[1];
        return;
        }
        Interval[1] = intervalR[1]; //otherwise, it's right upper bound

    }
    if (myNode->nodetype==2) { //Negation
        // create an interval for the child
        double interval_child[2];

        // calculate the interval for the child
        this->internalStoRI(timevec,meanTrace,covtrace,interval_child,myNode->left, CompleteTrace); 

        // Apply Rules
        Interval[0] = 1 - interval_child[1];
        Interval[1] = 1 - interval_child[0];
    }
    if (myNode->nodetype==6) { //Globally
        //Initialize stuff
        double interval_child[2];
        Interval[0] = 1;
        Interval[1] = 1;
        bool updated = false;

        for (int i = 0; i < timevec->size(); i++) { //for each time in the trace
            if (myNode->leftbound <= (*timevec)[i] && (*timevec)[i] <= myNode->rightbound) { //if the time is in the bounds of the operator
                updated = true; //report that we had data
                //Preprocess traces
                std::vector<double> time_adj(timevec->begin()+i, timevec->end()); //start with "current" element for all traces
                std::vector<Eigen::MatrixXd> mean_adj(meanTrace->begin()+i, meanTrace->end());
                std::vector<Eigen::MatrixXd> cov_adj(covtrace->begin()+i, covtrace->end());

                //subtract current time value from entire time vector (allows for handling of nested temporal operators)
                double timeval = time_adj[0];
                for (int j = 0; j < time_adj.size(); j++) {
                time_adj[j] -= timeval;
                }

                //TESTING - delete these later
                // std::cout << "\n\n\nCurrent time of exampination is: " << (*timevec)[i] << std::endl;
                // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, 19, CompleteTrace); 
                // std::cout << "Interval at node 19 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, 20, CompleteTrace); 
                // std::cout << "Interval at node 29 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, 21, CompleteTrace); 
                // std::cout << "Interval at node 21 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, 28, CompleteTrace); 
                // std::cout << "Interval at node 28 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, 29, CompleteTrace); 
                // std::cout << "Interval at node 29 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                //std::cout << "And the state is " << mean_adj[0] << "\n outside of the lower level calls\n";
                
                //calculate interval of child node
                this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, myNode->left, CompleteTrace); 
                
                //update bounds
                if (interval_child[0] < Interval[0]) { //update lower bound 
                Interval[0] = interval_child[0];
                }

                if (interval_child[1] < Interval[1]) { //update upper bound
                Interval[1] = interval_child[1];
                }
            }
        }

        if (!updated) { //report zero probability of success if we have no data in the time interval
        Interval[0] = 0;
        Interval[1] = 1;
        } else if ((timevec->back() < myNode->rightbound)) { //Report Lower bound of zero if there was data, but it's not finished yet
        Interval[0] = 0; 
        } 
    }
    if (myNode->nodetype==7) { //Eventually
        //Initialize stuff
        double interval_child[2];
        Interval[0] = 0;
        Interval[1] = 0;
        bool updated = false;

        for (int i = 0; i < timevec->size(); i++) { //for each time in the trace
            double deletethis = (*timevec)[i];
            if (myNode->leftbound <= (*timevec)[i] && (*timevec)[i] <= myNode->rightbound) { //if the time is in the bounds of the operator
                updated = true;
                //Preprocess traces
                std::vector<double> time_adj(timevec->begin()+i, timevec->end()); //start with "current" element for all traces
                std::vector<Eigen::MatrixXd> mean_adj(meanTrace->begin()+i, meanTrace->end());
                std::vector<Eigen::MatrixXd> cov_adj(covtrace->begin()+i, covtrace->end());

                //subtract current time value from entire time vector (allows for handling of nested temporal operators)
                //std::cout << "NEW ROOT TIMESTEP!!!" << std::endl;
                double timeval = time_adj[0];
                for (int j = 0; j < time_adj.size(); j++) {
                //std::cout << "\n\nTime Before: " << time_adj[j] << std::endl;
                time_adj[j] -= timeval;
                //std::cout << "Time After: " << time_adj[j] << std::endl;
                }
                
                //calculate interval of child node
                this->internalStoRI(&time_adj, &mean_adj, &cov_adj, interval_child, myNode->left, CompleteTrace);

                // std::cout << "time = " << (*timevec)[i] << std::endl;
                // std::cout << "Interval = [" << interval_child[0] << ", " << interval_child[1] << "] " << std::endl;  

                //update bounds
                if (interval_child[0] > Interval[0]) { //update lower bound 
                Interval[0] = interval_child[0];
                }

                if (interval_child[1] > Interval[1]) { //update upper bound
                Interval[1] = interval_child[1];
                }
            }
        }

        if (!updated) { //report zero probability of success if we have no data in the time interval
        Interval[0] = 0;
        Interval[1] = 1;
        } else if ((timevec->back() < myNode->rightbound) & !CompleteTrace) { //Report upper bound of one if there was data, but it's not finished yet
        Interval[1] = 1; //NOTE ON THIS CURRENT IMPLEMENTATION: This means that the trace must have more data than the time horizon to be considered "complete"
        }
    }
    if (myNode->nodetype==3) { //Until
        //Initialize stuff
        double leftinterval[2];
        double rightinterval[2];
        double globint[2];
        globint[0] = 1;
        globint[1] = 1;
        Interval[0] = 0;
        Interval[1] = 0;
        bool updated = false;

        // std::cout << "\n\nNEW CALL\n";

        for (int i = 0; i < timevec->size(); i++) { //for each time in the trace
            //Preprocess traces
            std::vector<double> time_adj(timevec->begin()+i, timevec->end()); //start with "current" element for all traces
            std::vector<Eigen::MatrixXd> mean_adj(meanTrace->begin()+i, meanTrace->end());
            std::vector<Eigen::MatrixXd> cov_adj(covtrace->begin()+i, covtrace->end());

            double timeval = time_adj[0];
            //subtract current time value from entire time vector (allows for handling of nested temporal operators)
            for (int j = 0; j < time_adj.size(); j++) {
                //std::cout << "\n\nTime Before: " << time_adj[j] << std::endl;
                time_adj[j] -= timeval;
                //std::cout << "Time After: " << time_adj[j] << std::endl;
            }

            //DELETE!
            // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, leftinterval, 11, true); 
            // std::cout << "\n\n Interval of 'trigger' region: [" << leftinterval[0] << ", " << leftinterval[1] << "]\n";
            // this->internalStoRI(&time_adj, &mean_adj, &cov_adj, leftinterval, 19, true); 
            // std::cout << "Interval of 'eventually' node: [" << leftinterval[0] << ", " << leftinterval[1] << "]\n";

            //calculate interval of left child node
            this->internalStoRI(&time_adj, &mean_adj, &cov_adj, leftinterval, myNode->left, CompleteTrace); // debug: is 'completetrace' true??
            // std::cout << "time: " << timeval<<std::endl;
            //update values for "globally/left" side
            if (leftinterval[0] < globint[0]) {
                globint[0] = leftinterval[0];
            }
            if (leftinterval[1] < globint[1]) {
                globint[1] = leftinterval[1];
            }

            // std::cout << "globally portion: [" << globint[0] << ", " << globint[1] << "\n";

            //calc function of satisfying temporal thingy at that time (right branch)
            if (myNode->leftbound <= (*timevec)[i] && (*timevec)[i] <= myNode->rightbound) { //if the time is in the bounds of the operator
                //calculate interval of right child node
                this->internalStoRI(&time_adj, &mean_adj, &cov_adj, rightinterval, myNode->right, CompleteTrace);
            }else if ((*timevec)[i] < myNode->leftbound){ //if time is less than lower bound
                rightinterval[0] = 0;
                rightinterval[1] = 0; //debug: check logic here
            }else { //time is greater than upper bound
                break;
            }
            
            //calc candidate interval of Until Operator
            double LB = rightinterval[0] + globint[0] - 1;
            double UB = globint[1];
            if (rightinterval[1] < globint[1]) {
                UB = rightinterval[1];
            }

            //update maxes of actual interval
            if (LB > Interval[0]) { //update lower bound 
                Interval[0] = LB;
            }

            if (UB > Interval[1]) { //update upper bound
                Interval[1] = UB;
            }
        }

        if ((timevec->back() < myNode->rightbound) & !CompleteTrace) { //Report upper bound of one if there was data, but it's not finished yet
            // std::cout << "time: " << timevec->back() << std::endl;
            // std::cout << "globint: [" << globint[0] << ", " << globint[1] << "] \n";
            // std::cout << "rightint: [" << rightinterval[0] << ", " << rightinterval[1] << "] \n";
            // std::cout << std::endl << std::endl;
            // std::cout << Interval[1] << std::endl;
            // std::cout << globint[1] << std::endl;
            if (globint[1] > Interval[1]) {
                Interval[1] = globint[1];
            }
        }

        // std::cout << "Interval: [" << Interval[0] << ", " << Interval[1] << "] " << std::endl;
    }
}

void PrSTL_Monitor::AriaMetric(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, bool CompleteTrace)
{
  this->internalStoRI(timevec,meanTrace,covtrace,Interval,this->myRootNode, CompleteTrace); 
}
