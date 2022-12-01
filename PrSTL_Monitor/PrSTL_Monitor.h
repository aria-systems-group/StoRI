#pragma once 

#include <vector>
#include <string>
#include <boost/math/special_functions/erf.hpp>
#include <eigen3/Eigen/Dense>
#include <map>

class ASTNode {
    public:
    	//general info
        int CLindex;
        int CRindex;
        int nodetype; //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for linear predicate (leaf node), 6 for "globally", 7 for "eventually"

        //for temporal operators
        double leftbound; //left time bound
        double rightbound; //right time bound

        //for predicates - predicate is satisfied when Ax - B > 0 
        Eigen::MatrixXd A; 
        float B; 

        ASTNode(int leftchildindex, int rightchildindex, int node_type, double lefttime, double righttime, Eigen::MatrixXd Avec, float Bcon) // constructor
        {
        	CLindex = leftchildindex;
        	CRindex = rightchildindex;
        	nodetype = node_type;

        	leftbound = lefttime;
        	rightbound = righttime;

        	A = Avec;
        	B = Bcon;
        }

};

class PrSTL_Monitor
{
    public:
        //Attributes
        std::vector<ASTNode> Mytree;

        std::map<int,std::vector<int>> heuristicInfo;

        std::vector<std::vector<double>> simpleHeuristicInfo;

        //Methods
        void BuildForm1(std::vector<ASTNode> *Mytree); //Function hardcodes a formula for now

        void BuildForm2(std::vector<ASTNode> *Mytree); //Function hardcodes a formula for now

        void BuildForm3(std::vector<ASTNode> *Mytree); //Function hardcodes a formula for now

        void BuildForm4(std::vector<ASTNode> *Mytree); //Function hardcodes a formula for now

        bool HyperplaneCCValidityChecker(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const;

        double HyperplaneProbabilityFinder(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const;

        void AriaMetric(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, int nodeind, bool CompleteTrace);

        bool isActive(double time, int nodeind);

        void CollectPredicates(int nodeind, std::vector<Eigen::MatrixXd> *A, std::vector<double> *b);

    //NOTE TO SELF: make constructor later. 
    //have it take in a tree and a state space dimension 
    //eventually, replace tree with formula and create parser method
};

