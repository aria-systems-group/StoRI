#pragma once 

#include <vector>
#include <string>
#include <boost/math/special_functions/erf.hpp>
#include <eigen3/Eigen/Dense>
#include <map>

class ASTNode {
    public:
      //general info
        ASTNode* left;
        ASTNode* right;
        int nodetype; //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for linear predicate (leaf node), 6 for "globally", 7 for "eventually"

      //for temporal operators
        double leftbound; //left time bound
        double rightbound; //right time bound

      //for predicates - predicate is satisfied when Ax - B > 0 
        Eigen::MatrixXd A; 
        float B; 

      //constructor
        ASTNode(int node_type) 
        {
        	nodetype = node_type;
          left = NULL;
          right = NULL;
        }
};

class StoRI_Monitor
{
    public:
        //Attributes
        ASTNode* myRootNode;

        std::map<int,std::vector<int>> heuristicInfo;

        std::vector<std::vector<double>> simpleHeuristicInfo;

        //Methods
        void BuildForm1(); //build formula 1 from paper

        void BuildForm2(); //build formula 2 from paper

        void BuildForm3(); //build formula 3 from paper

        ASTNode* BuildAST(std::string strFormula, std::map<std::string,ASTNode*> predicates);

        ASTNode* axisAlignedPredicate(int stateDim, int index, bool geq, double Bval);

        bool HyperplaneCCValidityChecker(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const;

        double HyperplaneProbabilityFinder(const Eigen::MatrixXd &A, const double &B, const Eigen::MatrixXd &X, const Eigen::MatrixXd &PX) const;

        void internalStoRI(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, ASTNode* myNode, bool CompleteTrace);

        void AriaMetric(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, bool CompleteTrace);

        bool isActive(double time, int nodeind);

        ASTNode* BuildAST(std::string strFormula, std::map<std::string,ASTNode> predicates);
};
