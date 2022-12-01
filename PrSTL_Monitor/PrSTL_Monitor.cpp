#include <iostream>
#include <ostream>
#include <vector>
#include "PrSTL_Monitor.h"
#include <map>

//TODO - something is off by a timestep or two, somewhere (look at online monitoring examples)
//TODO - make readme elaborating "allowed" operators
//TODO - chek formulas

//long term todos:
// - make parser
// - make branch - identifier 
// - make reach-conjunction-tree identifier 

void PrSTL_Monitor::BuildForm1(std::vector<ASTNode> *Mytree)
{
    //ARGUMENTS IN CONSTRUCTOR ARE LEFT IND, RIGHT IND, TYPE, LB, RB, A, B
    //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for predicate (leaf node), 6 for "globally", 7 for "eventually"
    //iota is zero if formula is x GREATER than value, or one if x LESS than value

    //create all the nodes
    ASTNode Rootnode(1,15,3,0,6,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(Rootnode);

    ASTNode node1(2,9,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node1);

    ASTNode node2(3,6,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node2);

    ASTNode node3(4,5,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node3);

    ASTNode node4(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
    Mytree->push_back(node4);

    ASTNode node5(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-4); // x < 4
    Mytree->push_back(node5);

    ASTNode node6(7,8,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node6);

    ASTNode node7(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-3); // y<3
    Mytree->push_back(node7);

    ASTNode node8(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),0); //y>0
    Mytree->push_back(node8);

    ASTNode node9(10,10,2,0,6,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node9);

    ASTNode node10(11,14,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node10);

    ASTNode node11(12,13,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node11);

    ASTNode node12(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),1); // x>1
    Mytree->push_back(node12);

    ASTNode node13(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-2); // x < 2
    Mytree->push_back(node13);

    ASTNode node14(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),2); // y > 2
    Mytree->push_back(node14);

    ASTNode node15(16,17,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node15);

    ASTNode node16(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),3); // x>3
    Mytree->push_back(node16);

    ASTNode node17(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),2); // y > 2
    Mytree->push_back(node17);

    heuristicInfo.insert(std::pair<int,std::vector<int>>(1,{2}));
    simpleHeuristicInfo.push_back({3.5,2.5});
}

void PrSTL_Monitor::BuildForm2(std::vector<ASTNode> *Mytree)
{
    //ARGUMENTS IN CONSTRUCTOR ARE LEFT IND, RIGHT IND, TYPE, LB, RB, A, B
    //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for predicate (leaf node), 6 for "globally", 7 for "eventually"
    //iota is zero if formula is x GREATER than value, or one if x LESS than value

    //create all the nodes
    ASTNode Rootnode(1,23,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(Rootnode);

    ASTNode node1(2,18,3,0,10,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node1);

    ASTNode node2(3,10,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node2);

    ASTNode node3(4,7,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node3);

    ASTNode node4(5,6,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node4);

    ASTNode node5(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
    Mytree->push_back(node5);

    ASTNode node6(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-4); // x < 4
    Mytree->push_back(node6);

    ASTNode node7(8,9,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node7);

    ASTNode node8(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),-2); //y > -2
    Mytree->push_back(node8);

    ASTNode node9(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-2); // y < 2
    Mytree->push_back(node9);

    ASTNode node10(11,11,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node10);

    ASTNode node11(12,15,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node11);

    ASTNode node12(13,14,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node12);

    ASTNode node13(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),-0.5); //y > -0.5
    Mytree->push_back(node13);

    ASTNode node14(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-0.5); // y < 0.5
    Mytree->push_back(node14);

    ASTNode node15(16,17,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node15);

    ASTNode node16(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),1.5); //x > 1.5
    Mytree->push_back(node16);

    ASTNode node17(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-3.5); //x < 3.5
    Mytree->push_back(node17);

    ASTNode node18(19,22,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node18);

    ASTNode node19(20,21,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node19);

    ASTNode node20(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),2); //x > 2
    Mytree->push_back(node20);

    ASTNode node21(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-3); // x < 3
    Mytree->push_back(node21);

    ASTNode node22(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),1); //y > 1
    Mytree->push_back(node22);

    ASTNode node23(2,24,3,0,10,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node23);

    ASTNode node24(25,28,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node24);

    ASTNode node25(26,27,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node25);

    ASTNode node26(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),2); //x > 2
    Mytree->push_back(node26);

    ASTNode node27(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-3); // x < 3
    Mytree->push_back(node27);

    ASTNode node28(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),1); //y < -1
    Mytree->push_back(node28);

    heuristicInfo.insert(std::pair<int,std::vector<int>>(2,{4}));
    heuristicInfo.insert(std::pair<int,std::vector<int>>(3,{11}));

    simpleHeuristicInfo.push_back({2.5,1.5});
    simpleHeuristicInfo.push_back({2.5,-1.5});
}

void PrSTL_Monitor::BuildForm3(std::vector<ASTNode> *Mytree)
{
    //ARGUMENTS IN CONSTRUCTOR ARE LEFT IND, RIGHT IND, TYPE, LB, RB, A, B
    //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for predicate (leaf node), 6 for "globally", 7 for "eventually"
    //iota is zero if formula is x GREATER than value, or one if x LESS than value

    //create all the nodes
    ASTNode Rootnode(1,14,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(Rootnode);

    ASTNode node1(2,9,3,0,5,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node1);

    ASTNode node2(3,6,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node2);

    ASTNode node3(4,5,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node3);

    ASTNode node4(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
    Mytree->push_back(node4);

    ASTNode node5(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-5); // x < 5
    Mytree->push_back(node5);

    ASTNode node6(7,8,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node6);

    ASTNode node7(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),0); //y > 0
    Mytree->push_back(node7);

    ASTNode node8(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-3); // y< 3
    Mytree->push_back(node8);

    ASTNode node9(10,13,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node9);

    ASTNode node10(11,12,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node10);

    ASTNode node11(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),1); //y > 1
    Mytree->push_back(node11);

    ASTNode node12(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-2); // y< 2
    Mytree->push_back(node12);

    ASTNode node13(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-1); // x < 1
    Mytree->push_back(node13);

    ASTNode node14(15,15,6,0,5,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node14);

    ASTNode node15(16,16,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node15);

    ASTNode node16(9,17,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node16);

    ASTNode node17(18,18,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node17);

    ASTNode node18(19,26,3,0,10,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node18);

    ASTNode node19(2,20,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node19);

    ASTNode node20(21,21,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node20);

    ASTNode node21(22,25,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node21);

    ASTNode node22(23,24,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node22);

    ASTNode node23(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),1); //x > 1
    Mytree->push_back(node23);

    ASTNode node24(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-4); // x < 4
    Mytree->push_back(node24);

    ASTNode node25(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),1); // y > 1
    Mytree->push_back(node25);

    ASTNode node26(27,28,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node26);

    ASTNode node27(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),4); // x > 4
    Mytree->push_back(node27);

    ASTNode node28(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-1); //y < 1
    Mytree->push_back(node28);

    heuristicInfo.insert(std::pair<int,std::vector<int>>(2,{4}));
    heuristicInfo.insert(std::pair<int,std::vector<int>>(3,{11}));

    simpleHeuristicInfo.push_back({0.5,1.5});
    simpleHeuristicInfo.push_back({4.5,0.5});
}

// void PrSTL_Monitor::BuildForm4(std::vector<ASTNode> *Mytree)
// {
//     //ARGUMENTS IN CONSTRUCTOR ARE LEFT IND, RIGHT IND, TYPE, LB, RB, A, B
//     //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for predicate (leaf node), 6 for "globally", 7 for "eventually"
//     //iota is zero if formula is x GREATER than value, or one if x LESS than value

//     //create all the nodes
//     ASTNode Rootnode(1,27,3,0,10,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(Rootnode);

//     ASTNode node1(2,9,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node1);

//     ASTNode node2(3,6,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node2);

//     ASTNode node3(4,5,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node3);

//     ASTNode node4(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
//     Mytree->push_back(node4);

//     ASTNode node5(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-3); // x < 3
//     Mytree->push_back(node5);

//     ASTNode node6(7,8,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node6);

//     ASTNode node7(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),0); //y > 0
//     Mytree->push_back(node7);

//     ASTNode node8(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-5); // y < 5
//     Mytree->push_back(node8);

//     ASTNode node9(10,10,2,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node9);

//     ASTNode node10(11,18,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node10);

//     ASTNode node11(12,15,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node11);

//     ASTNode node12(13,14,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node12);

//     ASTNode node13(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
//     Mytree->push_back(node13);

//     ASTNode node14(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-2.5); // x < 2.5
//     Mytree->push_back(node14);

//     ASTNode node15(16,17,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node15);

//     ASTNode node16(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),2); //y > 2
//     Mytree->push_back(node16);

//     ASTNode node17(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-3); // y < 3
//     Mytree->push_back(node17);

//     ASTNode node18(19,19,2,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node18);

//     ASTNode node19(20,20,7,0,3,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node19);

//     ASTNode node20(21,24,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node20);

//     ASTNode node21(22,23,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node21);

//     ASTNode node22(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
//     Mytree->push_back(node22);

//     ASTNode node23(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-1); // x < 1
//     Mytree->push_back(node23);

//     ASTNode node24(25,26,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node24);

//     ASTNode node25(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),4); //y > 4
//     Mytree->push_back(node25);

//     ASTNode node26(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-5); // y < 5
//     Mytree->push_back(node26);

//     ASTNode node27(28,31,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node27);

//     ASTNode node28(29,30,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node28);

//     ASTNode node29(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),2); //x > 2
//     Mytree->push_back(node29);

//     ASTNode node30(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-2.5); // x < 2.5
//     Mytree->push_back(node30);

//     ASTNode node31(32,33,1,0,0,Eigen::ArrayXd::Zero(3),0);
//     Mytree->push_back(node31);

//     ASTNode node32(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),4); //y > 4
//     Mytree->push_back(node32);

//     ASTNode node33(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-5); // y < 5
//     Mytree->push_back(node33);

//     simpleHeuristicInfo.push_back({2.5,4.5});
//     simpleHeuristicInfo.push_back({1.5,3.5});
// }

void PrSTL_Monitor::BuildForm4(std::vector<ASTNode> *Mytree)
{
    //ARGUMENTS IN CONSTRUCTOR ARE LEFT IND, RIGHT IND, TYPE, LB, RB, A, B
    //1 for "and", 2 for "not", 3 for "until", 4 for "true", 5 for predicate (leaf node), 6 for "globally", 7 for "eventually"
    //iota is zero if formula is x GREATER than value, or one if x LESS than value

    //create all the nodes
    ASTNode Rootnode(1,19,3,0,10,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(Rootnode);

    ASTNode node1(2,9,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node1);

    ASTNode node2(3,6,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node2);

    ASTNode node3(4,5,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node3);

    ASTNode node4(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),0); //x > 0
    Mytree->push_back(node4);

    ASTNode node5(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-3); // x < 3
    Mytree->push_back(node5);

    ASTNode node6(7,8,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node6);

    ASTNode node7(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),0); //y > 0
    Mytree->push_back(node7);

    ASTNode node8(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-5); // y < 5
    Mytree->push_back(node8);

    ASTNode node9(10,10,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node9);

    ASTNode node10(11,16,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node10);

    ASTNode node11(12,15,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node11);

    ASTNode node12(13,14,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node12);

    ASTNode node13(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),2); //y > 2
    Mytree->push_back(node13);

    ASTNode node14(-1,-1,5,0,0,Eigen::Vector4d(0,0,-1,0),-3); // y < 3
    Mytree->push_back(node14);

    ASTNode node15(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-2.5); // x < 2.5
    Mytree->push_back(node15);

    ASTNode node16(17,17,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node16);

    ASTNode node17(18,22,3,0,3,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node17);

    ASTNode node18(19,19,2,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node18);

    ASTNode node19(20,21,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node19);

    ASTNode node20(-1,-1,5,0,0,Eigen::Vector4d(1,0,0,0),2); //x > 2
    Mytree->push_back(node20);

    ASTNode node21(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),4); //y > 4
    Mytree->push_back(node21);

    ASTNode node22(23,24,1,0,0,Eigen::ArrayXd::Zero(3),0);
    Mytree->push_back(node22);

    ASTNode node23(-1,-1,5,0,0,Eigen::Vector4d(-1,0,0,0),-1); //x < 1
    Mytree->push_back(node23);

    ASTNode node24(-1,-1,5,0,0,Eigen::Vector4d(0,0,1,0),4); //y > 4
    Mytree->push_back(node24);

    simpleHeuristicInfo.push_back({2.5,4.5});
    simpleHeuristicInfo.push_back({0.5,4.5});
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

void PrSTL_Monitor::AriaMetric(std::vector<double> *timevec, std::vector<Eigen::MatrixXd> *meanTrace, std::vector<Eigen::MatrixXd> *covtrace, double *Interval, int nodeind, bool CompleteTrace)
{
    //NOTE: "until" is not coded up yet!! quite ugly ha ha
    if (this->Mytree[nodeind].nodetype==4) { //"True" Node
        Interval[0]= 1;
        Interval[1] = 1;
    }
    if (this->Mytree[nodeind].nodetype==5) { //Predicate Node
        double prob = this->HyperplaneProbabilityFinder(this->Mytree[nodeind].A,this->Mytree[nodeind].B, (*meanTrace)[0], (*covtrace)[0]);
        Interval[0] = prob;
        Interval[1] = prob;
    }
    if (this->Mytree[nodeind].nodetype==1) { //Conjunction
        // create intervals for the children
        double intervalL[2];
        double intervalR[2];

        // calculate intervals for the children
        this->AriaMetric(timevec,meanTrace,covtrace,intervalL,this->Mytree[nodeind].CLindex, CompleteTrace); 
        this->AriaMetric(timevec,meanTrace,covtrace,intervalR,this->Mytree[nodeind].CRindex, CompleteTrace); 

        //Apply Rules
        Interval[0] = intervalL[0] + intervalR[0] - 1; //lower bound

        if (Interval[0] < 0) { // account for negative probability
        Interval[0] = 0;
        }

        if (intervalL[1] < intervalR[1]) { //upperbound is left upper bound if it's smaller
        Interval[1] = intervalL[1];
        ////DELETE this print stuff
        //std::cout << "At node " << nodeind << ", Conjunction with nodes " << this->Mytree[nodeind].CLindex << " and " << this->Mytree[nodeind].CRindex << std::endl;
        //std::cout << "Interval is: [" << Interval[0] << ", " << Interval[1] << "] \n\n";
        return;
        }
        Interval[1] = intervalR[1]; //otherwise, it's right upper bound

        ////DELETE this print stuff
        //std::cout << "At node " << nodeind << ", Conjunction with nodes " << this->Mytree[nodeind].CLindex << " and " << this->Mytree[nodeind].CRindex << std::endl;
        //std::cout << "Interval is: [" << Interval[0] << ", " << Interval[1] << "] \n\n";
    }
    if (this->Mytree[nodeind].nodetype==2) { //Negation
        // create an interval for the child
        double interval_child[2];

        // calculate the interval for the child
        this->AriaMetric(timevec,meanTrace,covtrace,interval_child,this->Mytree[nodeind].CLindex, CompleteTrace); 

        // Apply Rules
        Interval[0] = 1 - interval_child[1];
        Interval[1] = 1 - interval_child[0];
    }
    if (this->Mytree[nodeind].nodetype==6) { //Globally
        //Initialize stuff
        double interval_child[2];
        Interval[0] = 1;
        Interval[1] = 1;
        bool updated = false;

        for (int i = 0; i < timevec->size(); i++) { //for each time in the trace
            if (Mytree[nodeind].leftbound <= (*timevec)[i] && (*timevec)[i] <= Mytree[nodeind].rightbound) { //if the time is in the bounds of the operator
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
                // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, 19, CompleteTrace); 
                // std::cout << "Interval at node 19 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, 20, CompleteTrace); 
                // std::cout << "Interval at node 29 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, 21, CompleteTrace); 
                // std::cout << "Interval at node 21 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, 28, CompleteTrace); 
                // std::cout << "Interval at node 28 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, 29, CompleteTrace); 
                // std::cout << "Interval at node 29 is: [" << interval_child[0] << ", " << interval_child[1] << "] \n";
                //std::cout << "And the state is " << mean_adj[0] << "\n outside of the lower level calls\n";
                
                //calculate interval of child node
                this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, this->Mytree[nodeind].CLindex, CompleteTrace); 
                
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
        } else if ((timevec->back() < this->Mytree[nodeind].rightbound)) { //Report Lower bound of zero if there was data, but it's not finished yet
        Interval[0] = 0; 
        } 
    }
    if (this->Mytree[nodeind].nodetype==7) { //Eventually
        //Initialize stuff
        double interval_child[2];
        Interval[0] = 0;
        Interval[1] = 0;
        bool updated = false;

        for (int i = 0; i < timevec->size(); i++) { //for each time in the trace
            double deletethis = (*timevec)[i];
            if (Mytree[nodeind].leftbound <= (*timevec)[i] && (*timevec)[i] <= Mytree[nodeind].rightbound) { //if the time is in the bounds of the operator
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
                this->AriaMetric(&time_adj, &mean_adj, &cov_adj, interval_child, this->Mytree[nodeind].CLindex, CompleteTrace);

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
        } else if ((timevec->back() < this->Mytree[nodeind].rightbound) & !CompleteTrace) { //Report upper bound of one if there was data, but it's not finished yet
        Interval[1] = 1; //NOTE ON THIS CURRENT IMPLEMENTATION: This means that the trace must have more data than the time horizon to be considered "complete"
        }
    }
    if (this->Mytree[nodeind].nodetype==3) { //Until
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
            // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, leftinterval, 11, true); 
            // std::cout << "\n\n Interval of 'trigger' region: [" << leftinterval[0] << ", " << leftinterval[1] << "]\n";
            // this->AriaMetric(&time_adj, &mean_adj, &cov_adj, leftinterval, 19, true); 
            // std::cout << "Interval of 'eventually' node: [" << leftinterval[0] << ", " << leftinterval[1] << "]\n";

            //calculate interval of left child node
            this->AriaMetric(&time_adj, &mean_adj, &cov_adj, leftinterval, this->Mytree[nodeind].CLindex, CompleteTrace); // debug: is 'completetrace' true??
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
            if (Mytree[nodeind].leftbound <= (*timevec)[i] && (*timevec)[i] <= Mytree[nodeind].rightbound) { //if the time is in the bounds of the operator
                //calculate interval of right child node
                this->AriaMetric(&time_adj, &mean_adj, &cov_adj, rightinterval, this->Mytree[nodeind].CRindex, CompleteTrace);
            }else if ((*timevec)[i] < Mytree[nodeind].leftbound){ //if time is less than lower bound
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

        if ((timevec->back() < this->Mytree[nodeind].rightbound) & !CompleteTrace) { //Report upper bound of one if there was data, but it's not finished yet
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

bool PrSTL_Monitor::isActive(double time, int nodeind)
{
    //todo: fix this if we add eventually conditions
    if ((this->Mytree[nodeind].nodetype == 6) || (this->Mytree[nodeind].nodetype == 7)) { //if temporal
        if ((time > this->Mytree[nodeind].rightbound) || (time > this->Mytree[nodeind].rightbound)) { // if time is outside of bounds
            return false;
        }
        return true;
    }else if ((this->Mytree[nodeind].nodetype == 5) || (this->Mytree[nodeind].nodetype == 4)) { //if predicate or truth 
        return true;
    }else if (this->Mytree[nodeind].nodetype == 1){ // conjunction
        bool leftbranch = this->isActive(time, this->Mytree[nodeind].CLindex);
        bool rightbranch = this->isActive(time, this->Mytree[nodeind].CRindex);
        if(leftbranch || rightbranch) {
            return true;
        }
        return false;
    }else { //everything else (negation)
        return this->isActive(time, this->Mytree[nodeind].CLindex);
    }
}

void PrSTL_Monitor::CollectPredicates(int nodeind, std::vector<Eigen::MatrixXd> *A, std::vector<double> *b)
{
    //YO this assumes it's being fed a CONJUNCTION TREE, so only nodes are conjunction and predicate
    if (this->Mytree[nodeind].nodetype == 5) { //predicate
        (*A).push_back(this->Mytree[nodeind].A);
        (*b).push_back(this->Mytree[nodeind].B);
    } else { //conjunction
        this->CollectPredicates(this->Mytree[nodeind].CLindex, A, b);
        this->CollectPredicates(this->Mytree[nodeind].CRindex, A, b);
    }
}