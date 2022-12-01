/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Texas A&M University
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
*   * Neither the name of the Texas A&M University nor the names of its
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

/* Authors: Qi Heng Ho (adapted from Saurav Agarwal) */

#ifndef R2BELIEF_SPACE_H_
#define R2BELIEF_SPACE_H_

// OMPL includes
#include "ompl/base/spaces/RealVectorStateSpace.h"

//other includes
#include <boost/math/constants/constants.hpp>
#include <vector>

//Eigen
#include <eigen3/Eigen/Dense>

using namespace ompl::base;
class R2BeliefSpace : public ompl::base::RealVectorStateSpace
{

    public:

        /** \brief A belief in R(4): (x, xdot, y, ydot, covariance) */
        class StateType : public RealVectorStateSpace::StateType
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            StateType(void) : RealVectorStateSpace::StateType()
            {
              
              sigma_ = 5.0*Eigen::MatrixXd::Identity(4,4);
              this->setLambda(Eigen::Matrix4d::Zero());

            }

            /** \brief Get the X component of the state */
            double getX(void) const
            {
                return this->values[0];
            }

            /** \brief Get the Xdot component of the state */
            double getXdot(void) const
            {
                return this->values[1];
            }

            /** \brief Get the Y component of the state */
            double getY(void) const
            {
                return this->values[2];
            }

            /** \brief Get the Y component of the state */
            double getYdot(void) const
            {
                return this->values[3];
            }

            /** \brief Get the mean vector of the state */
            const Eigen::Vector4d getMean(void) const
            {
                const Eigen::Vector4d stateVec(getX(), getXdot(), getY(), getYdot());
                return stateVec;
            }

            Eigen::Matrix4d getSigma(void) const
            {    
                return sigma_;
            }

            Eigen::Matrix4d getLambda(void) const
            {
                return lambda_;
            }

            Eigen::Matrix4d getCovariance(void) const
            {
                return sigma_ + lambda_;
            }

            /** \brief Set the X~ component of the state */
            void setX(double x)
            {
                this->values[0] = x;
            }

            /** \brief Set the Xdot component of the state */
            void setXdot(double xd)
            {
                this->values[1] = xd;
            }

            /** \brief Set the Y component of the state */
            void setY(double y)
            {
                this->values[2] = y;
            }

            /** \brief Set the Ydot component of the state */
            void setYdot(double yd)
            {
                this->values[3] = yd;
            }

            /** \brief Set the X and Y components of the state */
            void setMean(double x, double xd, double y, double yd)
            {
                setX(x);
                setXdot(xd);
                setY(y);
                setYdot(yd);
            }    

            void setMatrixData(const Eigen::Vector4d &x)
            {
                setX(x[0]);
                setXdot(x[1]);
                setY(x[2]);
                setYdot(x[3]);
            }

            void setSigmaX(double val){
                sigma_(0,0) = val;
            }

            void setSigmaXd(double val){
                sigma_(1,1) = val;
            }

            void setSigmaY(double val){
                sigma_(2,2) = val;
            }

            void setSigmaYd(double val){
                sigma_(3,3) = val;
            }

            void setSigma(Eigen::Matrix4d cov){
                sigma_ = cov;
            }

            void setSigma(double val){
                sigma_ = val*Eigen::MatrixXd::Identity(4,4);
            }

            void setLambda(Eigen::Matrix4d cov){
                lambda_ = cov;
            }

            Eigen::Vector4d getMatrixData(void) const
            {
                Eigen::Vector4d stateVec(getX(), getXdot(), getY(), getYdot());
                return stateVec;
            }

            void setCost(double cost){
                cost_ = cost;
            }

            double getCost(void) const{ 
                return cost_;
            }

            /** \brief Checks if the input state has stabilized to this state (node reachability check) */
            bool isReached(ompl::base::State *state, bool relaxedConstraint=false) const;

            static double meanNormWeight_, covNormWeight_, reachDist_;

            int DISTANCE_FUNCTION_TYPE_;
            
        private:
              Eigen::Matrix4d sigma_;
              Eigen::Matrix4d lambda_;
              int dimensions_ = 4;
              double cost_;

        };

        R2BeliefSpace(void) : RealVectorStateSpace(4)
        {
            setName("R2_BELIEF" + getName());
            type_ = STATE_SPACE_REAL_VECTOR;
        }

        virtual ~R2BeliefSpace(void)
        {
        }

        virtual State* allocState(void) const;

        virtual void copyState(State *destination,const State *source) const;

        virtual void freeState(State *state) const;

        virtual void copyToReals(std::vector<double> &reals, const State *source) const; //ADDED BY ROLAND

        //virtual void registerProjections(void);
        virtual double distance(const State* state1, const State *state2) const override;

        // gets the relative vector between "from" and "to"
        // equivalent to result = vectorA-vectorB
        void getRelativeState(const State *from, const State *to, State *state);

        void printBeliefState(const State *state);

};
#endif