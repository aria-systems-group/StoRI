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

#ifndef OMPL_CONTROL_PLANNERS_STLRRT_STLRRT_
#define OMPL_CONTROL_PLANNERS_STLRRT_STLRRT_

#include "PrSTL_Monitor.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include "ompl/datastructures/NearestNeighbors.h"
#include "R2BeliefSpace.h"
#include <ompl/base/spaces/TimeStateSpace.h>


#include <map> 

namespace ompl
{
    namespace control
    {
        /**
           @anchor cSTLRRT
           @par Short description
           STLRRT is a tree-based motion planner that uses the following
           idea: STLRRT samples a random state @b qr in the state space,
           then finds the state @b qc among the previously seen states
           that is closest to @b qr and expands from @b qc towards @b
           qr, until a state @b qm is reached. @b qm is then added to
           the exploration tree.
           This implementation is intended for systems with differential constraints.
           @par External documentation
           S.M. LaValle and J.J. Kuffner, Randomized kinodynamic planning, <em>Intl. J. of Robotics Research</em>, vol.
           20, pp. 378–400, May 2001. DOI: [10.1177/02783640122067453](http://dx.doi.org/10.1177/02783640122067453)<br>
           [[PDF]](http://ijr.sagepub.com/content/20/5/378.full.pdf)
           [[more]](http://msl.cs.uiuc.edu/~lavalle/STLRRTpubs.html)
        */

        /** \brief Rapidly-exploring Random Tree */
        class STLRRT : public base::Planner
        {
        public:
            /** \brief Constructor */
            STLRRT(const SpaceInformationPtr &si);

            ~STLRRT() override;

            /** \brief Continue solving for some amount of time. Return true if solution was found. */
            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            /** \brief Clear datastructures. Call this function if the
                input data to the planner has changed and you do not
                want to continue planning */
            void clear() override;

            /** \brief Set the StoRI constraint the planner is using */
            void setStoRIConstraint(double StoRIConstraint)
            {
                StoRIConstraint_=StoRIConstraint;
            }

            /** \brief Set the StoRI Monitor the planner is using */
            void setStoRIMonitor(PrSTL_Monitor StoRIMonitor)
            {
                MyMonitor_ = StoRIMonitor;
            }

            /** \brief Set the State Dynamics (For DIS calculations) */
            void setStateDyn(double N, double M, Eigen::MatrixXd A, Eigen::MatrixXd B)
            {
                StateDyn_N_ = N;
                StateDyn_M_ = M;
                StateDyn_A_ = A;
                StateDyn_B_ = B;
            }

            void setprintsoln(bool yayornay)
            {
                printsoln_ = yayornay;
            }

            void setResultContainer(double *container)
            {
                resultcontainer_ = container;
            }

            double* getResultContainer()
            {
                return resultcontainer_;
            }

            void optimizeSolution(bool yayornay)
            {
                AOversion_ = yayornay;
            }

            double getFirstScore()
            {
                return FirstScore_;
            }

            double getPathLength()
            {
                return SolnPathLength_;
            }

            /** In the process of randomly selecting states in the state
                space to attempt to go towards, the algorithm may in fact
                choose the actual goal state, if it knows it, with some
                probability. This probability is a real number between 0.0
                and 1.0; its value should usually be around 0.05 and
                should not be too large. It is probably a good idea to use
                the default value. */
            void setGoalBias(double goalBias)
            {
                goalBias_ = goalBias;
            }

            /** \brief Get the goal bias the planner is using */
            double getGoalBias() const
            {
                return goalBias_;
            }

            /** \brief Return true if the intermediate states generated along motions are to be added to the tree itself
             */
            bool getIntermediateStates() const
            {
                return addIntermediateStates_;
            }

            /** \brief Specify whether the intermediate states generated along motions are to be added to the tree
             * itself */
            void setIntermediateStates(bool addIntermediateStates)
            {
                addIntermediateStates_ = addIntermediateStates;
            }

            void getPlannerData(base::PlannerData &data) const override;

            /** \brief Set a different nearest neighbors datastructure */
            template <template <typename T> class NN>
            void setNearestNeighbors()
            {
                if (nn_ && nn_->size() != 0)
                    OMPL_WARN("Calling setNearestNeighbors will clear all states.");
                clear();
                nn_ = std::make_shared<NN<Motion *>>();
                setup();
            }

            void setup() override;

        protected:
            /** \brief Representation of a motion
                This only contains pointers to parent motions as we
                only need to go backwards in the tree. */
            class Motion
            {
            public:
                Motion() = default;

                /** \brief Constructor that allocates memory for the state and the control */
                Motion(const SpaceInformation *si)
                  : state(si->allocState()), control(si->allocControl())
                {
                }

                ~Motion() = default;

                /** \brief The state contained by the motion */
                base::State *state{nullptr};

                /** \brief The control contained by the motion */
                Control *control{nullptr};

                /** \brief The number of steps the control is applied for */
                unsigned int steps{0};

                /** \brief The parent motion in the exploration tree */
                Motion *parent{nullptr};

                // ADDED BY ROLAND
                
                /** \brief The time history of the signal. */
                std::vector<double> Motion_TimeSignal;

                /** \brief The state history of the signal. */
                std::vector<Eigen::MatrixXd> Motion_StateSignal;

                /** \brief The covariance history of the signal. */
                std::vector<Eigen::MatrixXd> Motion_CovSignal;
            };

            /** \brief Free the memory allocated by this planner */
            void freeMemory();

            /** \brief Compute distance between motions (actually distance between contained states) */
            double distanceFunction(const Motion *a, const Motion *b) const
            {
                return si_->distance(a->state, b->state);
            }

            /** \brief State sampler */
            base::StateSamplerPtr sampler_;

            /** \brief Control sampler */
            ControlSamplerPtr controlSampler_;

            /** \brief The base::SpaceInformation cast as control::SpaceInformation, for convenience */
            const SpaceInformation *siC_;

            /** \brief A nearest-neighbors datastructure containing the tree of motions */
            std::shared_ptr<NearestNeighbors<Motion *>> nn_;

            /** \brief The fraction of time the goal is picked as the state to expand towards (if such a state is
             * available) */
            double goalBias_{0.05};

            /** \brief Flag indicating whether intermediate states are added to the built tree of motions */
            bool addIntermediateStates_{false};

            /** \brief The random number generator */
            RNG rng_;

            /** \brief The most recent goal motion.  Used for PlannerData computation */
            Motion *lastGoalMotion_{nullptr};

            //HERE ARE THINGS I ADDED - roland

            /** \brief Whether or not to print and save final interval */
            bool printsoln_{false};

            /** \brief The number of times to sample controls when trying to find control that goes in good DIS */
            int ctrlK_{25}; //TODO: make function to adjust this

            /** \brief The bound for which the StoRI of the final trajectory must be above */
            double StoRIConstraint_{0.7};

            /** \brief The Monitor for the Stochastic Robustness Interval */
            PrSTL_Monitor MyMonitor_;

            /** \brief The System Dynamics (used for DIS calculations) [THIS ALGO ONLY WORKS FOR LINEAR SYSTEMS]*/
            int StateDyn_N_{0};

            int StateDyn_M_{0};

            Eigen::MatrixXd StateDyn_A_;

            Eigen::MatrixXd StateDyn_B_;
            
            unsigned int propagateMotionWhileValid(const Motion *start, const Control *control, int steps, Motion *result);

            bool isSignalValid(Motion *candidate, bool isfinished);

            void bouncetolist();

            double calcPathLength(Motion *leaf);

            double timehor{15}; //todo - make better

            bool saveTree_{true};

            double *resultcontainer_;

            bool AOversion_{false};

            double FirstScore_{0};

            double SolnPathLength_{0};
        };
    }
}

#endif