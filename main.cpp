// ROLAND ILYES

#include <memory>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/config.h>
#include <iostream>
#include <string>
#include <valarray>
#include <limits>
#include "stl_rrt.h"
#include <fstream>
#include "R2BeliefSpace.h"
#include <ompl/base/goals/GoalState.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include <ompl/base/Path.h>
#include <vector>
#include "PrSTL_Monitor.h"
#include <chrono>
#include "ompl/tools/benchmark/Benchmark.h"
#include <functional>

namespace ob = ompl::base;
namespace oc = ompl::control;

//todo: comment everything better

//to users: good idea to change between problems:
//          - state space bounds
//          - start state + covariance
//          - process noise Q
//          - formula to build
//          - stori constraint
//          - planning time
//          Can also change:
//              

// Discrete time propagation function
void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    //First, Define A and B Matrices
    Eigen::Matrix4d A;
    A << 0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0;
    Eigen::MatrixXd B(4,2);
    B << 0,0,1,0,0,0,0,1;

    //Next, Define Q Matrix
    Eigen::Matrix4d Q;
    Q << 0.01,0.001,0.001,0.001,
         0.001,0.01,0.001,0.001,
         0.001,0.001,0.01,0.001,
         0.001,0.001,0.001,0.01;

    //Todo: delete this, just make Q good to begin with
    Q = 0.001*Q; //should be 0.001

    //Next, construct Ahat matrix
    Eigen::MatrixXd Ahat(6,6);
    Ahat.topLeftCorner(4,4) = A;
    Ahat.topRightCorner(4,2) = B;
    Ahat.bottomLeftCorner(2,4) = Eigen::MatrixXd::Zero(2,4);
    Ahat.bottomRightCorner(2,2) = Eigen::MatrixXd::Zero(2,2);

    //Next, take matrix exponential
    Ahat = Ahat*duration;
    auto matex = Ahat.exp();

    //Next, extract F and G Matrices
    auto F = matex.block<4,4>(0,0);
    auto G = matex.block<4,2>(0,4);

    //Next, pull out the info you need from the inputs
    auto startvec = start->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getMean();
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    Eigen::Vector2d uvec(ctrl[0],ctrl[1]);

    //Now, calculate result
    Eigen::Vector4d resultvec = F*startvec + G*uvec;
    result->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setMatrixData(resultvec);

    //Propagate Uncertainty
    Eigen::Matrix4d resultcov = F*start->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance()*F.transpose() + Q;
    result->as<CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigma(resultcov);

    //Propatate Time
    result->as<CompoundStateSpace::StateType>()->as<TimeStateSpace::StateType>(1)->position = start->as<CompoundStateSpace::StateType>()->as<TimeStateSpace::StateType>(1)->position + duration;
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    //TODO -> make planner completely independent of this (right now, start state depends on this)
    return true;
}

/// @cond IGNORE
class DemoControlSpace : public oc::RealVectorControlSpace
{
public:

    DemoControlSpace(const ob::StateSpacePtr &stateSpace) : oc::RealVectorControlSpace(stateSpace, 2)
    {
    }
};
/// @endcond

void saveAndEvalSolution(oc::SimpleSetup *mySimpleSetup, PrSTL_Monitor *MyMonitor)
{
    std::ofstream(myfile);
    myfile.open("control_path.csv");

    //Here lies a tragic amount of postprocessing the path
    auto state_path = mySimpleSetup->getSolutionPath().getStates(); //split paths into state, controls, control durations
    auto control_path = mySimpleSetup->getSolutionPath().getControls();
    auto control_duration_path = mySimpleSetup->getSolutionPath().getControlDurations();

    std::vector<std::vector<double>> total_info_vec;

    std::vector<double> info_row;

    //Also, make vectors needed by metric
    std::vector<double> time_vector;
    std::vector<Eigen::MatrixXd> mean_trace;
    std::vector<Eigen::MatrixXd> cov_trace;

    //for each state
    for (int i = 0; i < mySimpleSetup->getSolutionPath().getStateCount(); i++) {
        //Cast states to types we expect
        auto *wackstate = state_path[i]->as<ob::CompoundStateSpace::StateType>(); //cast to compound state
        auto *beliefstate = wackstate->as<R2BeliefSpace::StateType>(0);
        auto *timestate = wackstate->as<TimeStateSpace::StateType>(1);

        //Add state info to vector
        auto beliefs(std::make_shared<R2BeliefSpace>());
        beliefs->copyToReals(info_row, beliefstate);

        //Add control info to vector
        if (i>0) {
            //Cast controls to types we expect
            auto *controlinput = control_path[i-1]->as<DemoControlSpace::ControlType>();
            info_row.push_back(controlinput->values[0]);
            info_row.push_back(controlinput->values[1]);

            //Add control duration info to vector
            info_row.push_back(control_duration_path[i-1]);
        } else {
            info_row.push_back(0);
            info_row.push_back(0);
            info_row.push_back(0);
        }

        //Add time to vector
        info_row.push_back(timestate->position);

        //Add to csv
        for(int j = 0; j < info_row.size(); j++) {
            myfile << info_row[j] << ", ";
        }
        myfile << std::endl;

        //stuff for metric
        time_vector.push_back(timestate->position);
        mean_trace.push_back(beliefstate->getMean());
        cov_trace.push_back(beliefstate->getCovariance());
    }

    //ss.getSolutionPath().printAsMatrix(myfile);
    myfile.close();
}

void optionalPostRunEvent(const ompl::base::PlannerPtr &planner, ompl::tools::Benchmark::RunProperties &run)
{
    // saving additional data after a run

    auto score = planner->as<oc::STLRRT>()->getResultContainer();

    auto length = planner->as<oc::STLRRT>()->getPathLength();
 
    run["Final Score"] = std::to_string(score[0]);

    run["Path Length"] = std::to_string(length);
}

void planWithSimpleSetup(int formula, bool benchmark, bool optimize)
{
    /// construct the state space we are planning in
    auto beliefs(std::make_shared<R2BeliefSpace>());
    auto times(std::make_shared<TimeStateSpace>());

    auto space = beliefs + times;

    //lower weight on time space for distance calculations (default weight is 1)
    space->as<ompl::base::CompoundStateSpace>()->setSubspaceWeight(1,0.25); //TODO: determine if you want to weigh time as well 

    /// set the bounds for the state space (todo: make realistic later)
    ob::RealVectorBounds bounds(4);
    if (formula == 1) {
        bounds.setLow(0,0); //x position bounds
        bounds.setHigh(0,4);
        bounds.setLow(2,0); //y position bounds
        bounds.setHigh(2,3);
    } else if (formula == 2) {
        bounds.setLow(0,0); //x position bounds
        bounds.setHigh(0,4);
        bounds.setLow(2,-2); //y position bounds
        bounds.setHigh(2,2);
    }else if (formula == 3) {
        bounds.setLow(0,0); //x position bounds
        bounds.setHigh(0,5);
        bounds.setLow(2,0); //y position bounds
        bounds.setHigh(2,3);
    }else if (formula == 4) {
        bounds.setLow(0,0); //x position bounds
        bounds.setHigh(0,3);
        bounds.setLow(2,0); //y position bounds
        bounds.setHigh(2,5);
    }
    
    bounds.setLow(1,-2); //x velocity bounds
    bounds.setHigh(1,2);

    bounds.setLow(3,-2); //y velocity bounds
    bounds.setHigh(3,2);

    beliefs->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<DemoControlSpace>(space));

    // set the bounds for the control space (todo: make realistic later)
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-2);
    cbounds.setHigh(2);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set state validity checking for this space
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); });

    // set state propagator
    ss.setStatePropagator(propagate);
    si->setPropagationStepSize(0.15);
    si->setMinMaxControlDuration(1, 10);

    /// create a start state
    ob::ScopedState<> start(space);
    if (formula == 1) {
        start[0] = 0.5;
        start[1] = 0.1;
        start[2] = 2.5;
        start[3] = 0;
    }else if (formula == 2) {
        start[0] = 0.5;
        start[1] = 0.1;
        start[2] = 1.5;
        start[3] = 0;
    }else if (formula == 3) {
        start[0] = 2.5;
        start[1] = -0.1;
        start[2] = 2.5;
        start[3] = 0;
    }else if (formula == 4) {
        start[0] = 1.5;
        start[1] = 0.5;
        start[2] = 1;
        start[3] = 0.5;
    }

    start[4] = 0; //time

    Eigen::Matrix4d start_cov;
    start_cov << 0,0,0,0,
                 0,0,0,0,
                 0,0,0,0,
                 0,0,0,0;
    
    start->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigma(start_cov);

    /// create a  goal state (not used so don't think too hard about it)
    ob::ScopedState<> goal(space);
    goal[0] = 2.5;
    goal[1] = 0.5;
    goal[2] = 1.5;
    goal[3] = -0.5;
    goal[4] = 5;
    goal->as<ob::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->setSigma(start_cov); //todo: what to put here? 

    /// set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.05); //threshold doesn't matter

    // set the sst planner
    oc::SpaceInformationPtr si_p = ss.getSpaceInformation();

    // code up dynamics to give to planner
    Eigen::Matrix4d sysA;
    sysA << 0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0;
    Eigen::MatrixXd sysB(4,2);
    sysB << 0,0,1,0,0,0,0,1;

    // initialte planner and planner info
    oc::STLRRT roplan(si_p);
    if (optimize) {
        roplan.setStoRIConstraint(0.0);
    }else {
        roplan.setStoRIConstraint(0.9); // constraint on StoRI for path to be considered satisfactory
    }
    
    roplan.setStateDyn(4,2, sysA, sysB); // give state dynamics of system
    roplan.setGoalBias(0.1); //bias for how often to use heuristic
    roplan.setprintsoln(true); //if true, prints final solution's interval to output and CSV
    PrSTL_Monitor MyMonitor; //Initial monitor
    if (formula == 1) {//which (hardcoded) formula to solve for
        MyMonitor.BuildForm1(); 
    }else if (formula == 2) {
        MyMonitor.BuildForm2(); 
    }else if (formula == 3) {
        MyMonitor.BuildForm3(); 
    }else if (formula == 4) {
        MyMonitor.BuildForm4(); 
    }
    roplan.optimizeSolution(optimize);
    roplan.setStoRIMonitor(MyMonitor); //give that monitor to the planner
    double result[2];
    roplan.setResultContainer(result);
    ob::PlannerPtr planner(&roplan); //create pointer to planner
    ss.setPlanner(planner); //give planner to ss

    ss.setup();

    if (benchmark) {
        roplan.setprintsoln(false);
        std::string str1;
        std::string str2;
        if (formula == 1) {
            str1 = "Formula1";
        }else if (formula == 2) {
            str1 = "Formula2";
        }else if (formula == 3) {
            str1 = "Formula3";
        }else if (formula == 4) {
            str1 = "Formula4";
        }

        if (optimize) {
            str2 = "_AO";
        } else {
            str2 = "_RRT";
        }

        std::string benchfile = str1+str2+"_HeuristicON.log";

        // First we create a benchmark class:
        ompl::tools::Benchmark b(ss, benchfile);

        // For planners that we want to configure in specific ways,
        // the ompl::base::PlannerAllocator should be used:
        b.addPlanner(planner);
        // etc.
        b.setPostRunEvent(std::bind(&optionalPostRunEvent, std::placeholders::_1, std::placeholders::_2));

        // Now we can benchmark: 5 second time limit for each plan computation,
        // 100 MB maximum memory usage per plan computation, 50 runs for each planner
        // and true means that a text-mode progress bar should be displayed while
        // computation is running.
        ompl::tools::Benchmark::Request req;
        req.maxTime = 300.0;
        req.maxMem = 2048.0; //2 gb
        req.runCount = 10;
        req.displayProgress = true;
        b.benchmark(req);

        // This will generate a file of the form ompl_host_time.log
        b.saveResultsToFile(benchfile.c_str());
    }
    
    if (!benchmark) {
        ob::PlannerStatus solved = ss.solve(300);

        if (solved)
        {
            std::cout << "Found solution!" << std::endl;

            std::cout << "First Score: " << roplan.getFirstScore() << std::endl;

            saveAndEvalSolution(&ss, &MyMonitor);

        }
        else
            std::cout << "No solution found" << std::endl;
    }
        
}

void evaluateMatlabTrace()
{
    //initialize vectors
    std::vector<double> timevec;
    std::vector<Eigen::MatrixXd> meanTrace;
    std::vector<Eigen::MatrixXd> covtrace;
    std::vector<double> meanvec;
    std::vector<double> covvec;

    //load in the data from csv
    std::ifstream myfile1;
    std::ofstream myfile;
    myfile1.open ("/Users/rolandilyes/PrSTL_Monitor/src/build/Matlab_path_data.csv");
    std::string line;
    std::string numb;

    int ind_1 = 0;
    int ind_2 = 0;
    float test;
    std::string record;
    while (std::getline(myfile1, record)){
        std::istringstream line(record);
        while (std::getline(line,record, ',')){
            //first, sort elements to temporary vectors
            if (ind_2 == 0) { //if first element, save to time vector
            timevec.push_back(std::stod(record));
            } else if (ind_2 > 0 & ind_2 < 5) { //next four elements are mean
            meanvec.push_back(std::stod(record));
            } else { //rest are covariance
            covvec.push_back(std::stod(record));
            }

            //iterate counter
            ind_2++;
        }

        //next, add Eigen Matrices to the global traces
        meanTrace.push_back(Eigen::Vector4d(meanvec[0],meanvec[1],meanvec[2],meanvec[3]));
        auto cov = new Eigen::Matrix4d;
        (*cov) << covvec[0], covvec[1], covvec[2], covvec[3],
                  covvec[1], covvec[4], covvec[5], covvec[6],
                  covvec[2], covvec[5], covvec[7], covvec[8],
                  covvec[3], covvec[6], covvec[8], covvec[9];
        covtrace.push_back((*cov));
        delete cov;

        //clear temporary vectors
        meanvec.clear();
        covvec.clear();
        
        ind_2 = 0;
        ind_1++;
    }

    myfile1.close();

    //evaluate the trace
    PrSTL_Monitor MyMonitor;
    MyMonitor.BuildForm1();
    double Interval[2];

    // std::vector<double> temptime;
    // std::vector<Eigen::MatrixXd> tempmean;
    // std::vector<Eigen::MatrixXd> tempcov;
    for (int i = 0; i < timevec.size(); i++) {
        auto temptime = timevec;
        temptime.resize(i);

        auto tempmean = meanTrace;
        tempmean.resize(i);

        auto tempcov = covtrace;
        tempcov.resize(i);

        // temptime.push_back(timevec[i]);
        // tempmean.push_back(meanTrace[i]);
        // tempcov.push_back(covtrace[i]);

        MyMonitor.AriaMetric(&temptime, &tempmean, &tempcov, Interval, false);

        std::cout << "Interval at time " << timevec[i] << ": [" << Interval[0] << ", " << Interval[1] << "] \n";

        temptime.clear();
        tempmean.clear();
        tempcov.clear();
    }

    MyMonitor.AriaMetric(&timevec, &meanTrace, &covtrace, Interval, true);

    std::cout << "\n\n\n FINAL INTERVAL IS: [" << Interval[0] << ", " << Interval[1] << "] " << std::endl;

    myfile.open("Interval.csv");
    myfile << Interval[0] << ", " << Interval[1] << std::endl;
    myfile.close();
}

int main(int argc, char ** argv)
{
    int formula = std::stoi(argv[1]);

    bool benchmark;

    bool optimize;

    if (argc == 2) {
        benchmark = false;
        optimize = false;
    } else if (argc == 3) {
        optimize = std::stoi(argv[2]);
        benchmark = false;
    } else if (argc == 4) {
        optimize = std::stoi(argv[2]);
        benchmark = std::stoi(argv[3]);
    }

    // int formula = 1;
    // bool benchmark = true;
    // bool optimize = false;

    // planWithSimpleSetup(formula,benchmark,optimize);

    // evaluateMatlabTrace();

    // build predicates
    std::map<std::string,ASTNode*> mymap;

    PrSTL_Monitor myMonitor; //Initial monitor

    ASTNode* firstNode = new ASTNode(5);
    firstNode->A = Eigen::Vector4d(1,0,0,0);
    firstNode->B = 3;
    mymap["x>3"] = firstNode;

    ASTNode* secondNode = new ASTNode(5);
    secondNode->A = Eigen::Vector4d(-1,0,0,0);
    secondNode->B = -4;
    mymap["x<4"] = secondNode;
    
    ASTNode* thirdNode = new ASTNode(5);
    thirdNode->A = Eigen::Vector4d(0,0,1,0);
    thirdNode->B = 2;
    mymap["y>2"] = thirdNode;

    ASTNode* fourthNode = new ASTNode(5);
    fourthNode->A = Eigen::Vector4d(0,0,-1,0);
    fourthNode->B = -3;
    mymap["y<3"] = fourthNode;

    ASTNode* fifthNode = new ASTNode(5);
    fifthNode->A = Eigen::Vector4d(1,0,0,0);
    fifthNode->B = 1;
    mymap["x>1"] = fifthNode;

    ASTNode* sixthNode = new ASTNode(5);
    sixthNode->A = Eigen::Vector4d(-1,0,0,0);
    sixthNode->B = -2;
    mymap["x<2"] = sixthNode;

    //build regions
    ASTNode* goal = myMonitor.BuildAST("((x>3)&(x<4))&((y>2)&(y<3))",mymap);
    mymap["goal"] = goal;

    ASTNode* obstacle = myMonitor.BuildAST("((x>1)&(x<2))&((y>2)&(y<3))",mymap);
    mymap["obstacle"] = obstacle;

    //build specification
    ASTNode* testformula = myMonitor.BuildAST("(!(obstacle))U[4,6](goal)",mymap);

    return 0;
}
