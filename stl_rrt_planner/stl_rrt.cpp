/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include "stl_rrt.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>
#include <iostream>
#include "PrSTL_Monitor.h"
#include "R2BeliefSpace.h" //todo: remove the need for explicit decalring statespace types in this script
#include <ompl/base/spaces/TimeStateSpace.h>

#include <unsupported/Eigen/MatrixFunctions>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <fstream>
#include <algorithm>
#include <map>
#include <fstream>

// all the to-do's are in the stl_sst files (not the STLRRT files)

ompl::control::STLRRT::STLRRT(const SpaceInformationPtr &si) : base::Planner(si, "STLRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &STLRRT::setGoalBias, &STLRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &STLRRT::setIntermediateStates, &STLRRT::getIntermediateStates,
                                "0,1");
}

ompl::control::STLRRT::~STLRRT()
{
    freeMemory();
}

void ompl::control::STLRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::STLRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::STLRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::control::STLRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        nn_->add(motion);

        //Added by roland
        motion->Motion_TimeSignal.push_back(st->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::TimeStateSpace::StateType>(1)->position); 
        motion->Motion_StateSignal.push_back(st->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getMean()); 
        motion->Motion_CovSignal.push_back(st->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance());
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler(); //note: changed from default rrt directed control sampler to accomodate for new validity checking

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    unsigned iterations = 0;

    bool solved = false;

    auto path(std::make_shared<PathControl>(si_));

    //initialize result container to proper values;
    resultcontainer_[0] = 0;
    resultcontainer_[1] = 1;

    while (ptc == false)
    {
        // std::cout << "HI MOM " << iterations << std::endl;

        double myrand = rng_.uniform01(); // generate random number for heuristics purposes

        //generate state and time
        sampler_->sampleUniform(rstate); 
        
        double timesample = rng_.uniform01()*timehor;
        rstate->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::TimeStateSpace::StateType>(1)->position = timesample;

        if (rng_.uniform01() <= goalBias_) { // simple heuristic
            //randomly select goal
            int selectedgoal = rng_.uniformInt(0, (MyMonitor_.simpleHeuristicInfo.size()-1)); //randomly pick a "goal"
            
            rstate->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[0] = MyMonitor_.simpleHeuristicInfo[selectedgoal][0];
            rstate->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->values[2] = MyMonitor_.simpleHeuristicInfo[selectedgoal][1];
        } 

        // find closest state in the tree TODO: make sure node selection uses time as well
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = rng_.uniformInt(siC_->getMinControlDuration(),siC_->getMaxControlDuration());
        controlSampler_->sample(rctrl);
        unsigned int propCd = propagateMotionWhileValid(nmotion, rctrl, cd, rmotion);

        if (propCd >= siC_->getMinControlDuration()) //NOTE: different from SST version, doesn't have to prop all the way to cd
        {
            /* create a motion */
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, rmotion->state);
            siC_->copyControl(motion->control, rctrl);
            motion->steps = cd;
            motion->parent = nmotion;

            //added by roland - copy trace info into new motion as well
            motion->Motion_TimeSignal = rmotion->Motion_TimeSignal;
            motion->Motion_StateSignal = rmotion->Motion_StateSignal;
            motion->Motion_CovSignal = rmotion->Motion_CovSignal;

            nn_->add(motion);
            double dist = 0.0;
            bool solv = isSignalValid(motion, true); //check to see if signal is complete and satisfactory
            if (solv)
            {
                // find interval of solution
                MyMonitor_.AriaMetric(&motion->Motion_TimeSignal, &motion->Motion_StateSignal, &motion->Motion_CovSignal, resultcontainer_, true); 

                // save as first score if one hasn't been found yet
                if (FirstScore_ == 0) {
                    FirstScore_ = resultcontainer_[0];
                }

                // save as solution (should ALWAYS improve StoRI in this framework)
                approxdif = dist;
                solution = motion;

                // calc path length (benchmarking ugh)
                SolnPathLength_ = calcPathLength(solution);

                lastGoalMotion_ = solution;
                /* construct the solution path */
                std::vector<Motion *> mpath;
                while (solution != nullptr)
                {
                    mpath.push_back(solution);
                    solution = solution->parent;
                }

                /* set the solution path */
                auto path2(std::make_shared<PathControl>(si_));
                for (int i = mpath.size() - 1; i >= 0; --i)
                    if (mpath[i]->parent)
                        path2->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
                    else
                        path2->append(mpath[i]->state);
                solved = true;
                path = path2;

                // reset planner, adjust StoRI bound if optimizing version
                if (AOversion_) {
                    //clear function stuff
                    sampler_.reset();
                    controlSampler_.reset();

                    nn_->remove(lastGoalMotion_); //remove motion! so that it doesn't get deleted here and we can still access it later

                    std::vector<Motion *> motions;
                    nn_->list(motions);
                    for (int i = 1;i < motions.size();i++)
                    {
                        nn_->remove(motions[i]);
                        if (motions[i]->state)
                            si_->freeState(motions[i]->state);
                        if (motions[i]->control)
                            siC_->freeControl(motions[i]->control);
                        delete motions[i];
                    }
                    
                    if (!sampler_)
                        sampler_ = si_->allocStateSampler();
                    if (!controlSampler_)
                        controlSampler_ = siC_->allocControlSampler(); //note: changed from default rrt directed control sampler to accomodate for new validity checking

                    //print stuff
                    std::cout << nn_->size() << std::endl;
                    std::cout << "Found Solution with score of " << resultcontainer_[0] << std::endl;
                    //set new constraint
                    setStoRIConstraint(resultcontainer_[0]);
                } else { //terminate
                    break;
                }
            }
        }

        iterations++;
    }

    pdef_->addSolutionPath(path, false, approxdif, getName());

    if(saveTree_)
    {
        bouncetolist();
    }

    bool approximate = false;

    if (lastGoalMotion_ != nullptr)
    {
        //save finer resolution data
        solution = lastGoalMotion_;
        std::ofstream(myfile);
        myfile.open("cov_dat.csv");

        std::vector<double> info_row;

        //Also, make vectors needed by metric
        auto time_vector = solution->Motion_TimeSignal;
        auto mean_trace = solution->Motion_StateSignal;
        auto cov_trace = solution->Motion_CovSignal;

        //for each state
        for (int i = 0; i < time_vector.size(); i++) { //for each point
            Eigen::MatrixXd stateattime = mean_trace[i];
            Eigen::MatrixXd covattime = cov_trace[i];

            //Add state info to vector
            info_row.push_back(stateattime(0,0));
            info_row.push_back(stateattime(1,0));
            info_row.push_back(stateattime(2,0));
            info_row.push_back(stateattime(3,0));

            //Add covariance to vector
            info_row.push_back(covattime(0,0));
            info_row.push_back(covattime(0,1));
            info_row.push_back(covattime(0,2));
            info_row.push_back(covattime(0,3));
            info_row.push_back(covattime(1,1));
            info_row.push_back(covattime(1,2));
            info_row.push_back(covattime(1,3));
            info_row.push_back(covattime(2,2));
            info_row.push_back(covattime(2,3));
            info_row.push_back(covattime(3,3));

            //Add time to vector
            info_row.push_back(time_vector[i]);

            //Add to csv
            for(int j = 0; j < info_row.size(); j++) {
                myfile << info_row[j] << ", ";
            }
            myfile << std::endl;

            info_row.clear();
        }

        //ss.getSolutionPath().printAsMatrix(myfile);
        myfile.close();

        std::cout << "TESTING PATH LENGTH: " << calcPathLength(solution) << std::endl;

        if (printsoln_) {
            //resultcontainer should containt best solution so far

            std::cout << "\n\n\n FINAL INTERVAL IS: [" << resultcontainer_[0] << ", " << resultcontainer_[1] << "] " << std::endl;

            myfile.open("Interval.csv");
            myfile << resultcontainer_[0] << ", " << resultcontainer_[1] << std::endl;
            myfile.close();
        }
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::STLRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}

unsigned int ompl::control::STLRRT::propagateMotionWhileValid(const Motion *start, const Control *control, int steps, Motion *result) 
{
    //Modified from OMPL's propagateWhileValid function
    if (steps == 0)
    {
        if (result->state != start->state)
            siC_->copyState(result->state, start->state);
        return 0;
    }
 
    //Performing some step size magic
    double signedStepSize = steps > 0 ? siC_->getPropagationStepSize() : -siC_->getPropagationStepSize(); 
    steps = abs(steps);

    //Make copy of parent motion's traces TODO - look into memcpy or other, more efficient method 
    result->Motion_TimeSignal = start->Motion_TimeSignal;
    result->Motion_StateSignal = start->Motion_StateSignal;
    result->Motion_CovSignal = start->Motion_CovSignal;
 
    // perform the first step of propagation
    siC_->getStatePropagator()->propagate(start->state, control, signedStepSize, result->state);

    // add resulting covariance, time, and mean to traces 
    result->Motion_TimeSignal.push_back(result->state->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::TimeStateSpace::StateType>(1)->position); 
    result->Motion_StateSignal.push_back(result->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getMean()); 
    result->Motion_CovSignal.push_back(result->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance());

    // if we found a solution after one step, stop
    if (isSignalValid(result,true))
    {
        return 1;
    }
    // if we found a valid state after one step, we can go on
    if (isSignalValid(result,false))
    {
        base::State *temp1 = result->state;
        base::State *temp2 = siC_->allocState(); 
        base::State *toDelete = temp2;
        unsigned int r = steps;
 
        // for the remaining number of steps
        for (int i = 1; i < steps; ++i)
        {
            siC_->getStatePropagator()->propagate(temp1, control, signedStepSize, temp2);
            result->Motion_TimeSignal.push_back(temp2->as<ompl::base::CompoundStateSpace::StateType>()->as<ompl::base::TimeStateSpace::StateType>(1)->position); 
            result->Motion_StateSignal.push_back(temp2->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getMean()); 
            result->Motion_CovSignal.push_back(temp2->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getCovariance());

            if (isSignalValid(result,true)) //break early if found solution
            {
                std::swap(temp1, temp2);
                r = i+1; //debug: this true? 
                break;
            }
            if (isSignalValid(result,false)) 
                std::swap(temp1, temp2);
            else
            {
                // remove the newly added elements of the traces
                result->Motion_TimeSignal.pop_back();
                result->Motion_StateSignal.pop_back();
                result->Motion_CovSignal.pop_back();
                // the last valid state is temp1;
                r = i;
                break;
            }
        }
 
        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure result contains that information
        if (result->state != temp1)
            siC_->copyState(result->state, temp1);
 
        // free the temporary memory
        siC_->freeState(toDelete);

        //TODO - would it help memory to explicity clear data in rmotion at any point? 
 
        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    if (result->state != start->state)
        siC_->copyState(result->state, start->state);
    return 0;
}

bool ompl::control::STLRRT::isSignalValid(Motion *candidate, bool isfinished)
{
    //Define interval, calculate StoRI (partial or complete depending on isfinished)
    double Interval[2];
    MyMonitor_.AriaMetric(&candidate->Motion_TimeSignal, &candidate->Motion_StateSignal, &candidate->Motion_CovSignal, Interval, isfinished); 

    if (!isfinished && (Interval[1] > StoRIConstraint_)) {
        //std::cout << "Partial Interval of [" << Interval[0] << ", " << Interval[1] << "] \n";
        return true;
    }else if (isfinished && (Interval[0] > StoRIConstraint_)) {
        // std::cout << "valid final trace with interval [" << Interval[0] << ", " << Interval[1] << "] \n";
        return true;
    }
    return false;
}

void ompl::control::STLRRT::bouncetolist()
{
    //Gather list of motions from nn structure
    std::vector<Motion *> mylist;
    nn_->list(mylist);

    //begin stream
    std::ofstream(myfile);
    myfile.open("tree_data.csv");
    std::vector<double> info_row;

    //save EDGES - so position and parent's position
    for (int i = 0; i < mylist.size(); i++) {
        //save motion's position
        info_row.push_back(mylist[i]->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX());
        info_row.push_back(mylist[i]->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY());

        if (mylist[i]->parent!=nullptr) { //if motion has a parent, add its position as well
            info_row.push_back(mylist[i]->parent->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX());
            info_row.push_back(mylist[i]->parent->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY());
        }else { //if not, just put original position again
            info_row.push_back(mylist[i]->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getX());
            info_row.push_back(mylist[i]->state->as<ompl::base::CompoundStateSpace::StateType>()->as<R2BeliefSpace::StateType>(0)->getY());
        }

        //write to csv
        for(int j = 0; j < info_row.size(); j++) {
            myfile << info_row[j] << ", ";
        }
        myfile << std::endl;

        info_row.clear();
    }
    myfile.close();
}

double ompl::control::STLRRT::calcPathLength(Motion *leaf)
{
    auto path = leaf->Motion_StateSignal;
    double distance = 0;
    double dx;
    double dy;
    Eigen::MatrixXd state1;
    Eigen::MatrixXd state2;
    for (int i = 1; i < path.size(); i++) {
        state1 = path[i-1];
        state2 = path[i];
        dx = state2(0,0) - state1(0,0);
        dy = state2(2,0) - state1(2,0);
        distance += pow(dx*dx+dy*dy,0.5);
    }
    return distance;
}
