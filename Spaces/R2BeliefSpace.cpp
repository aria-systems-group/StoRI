#include "R2BeliefSpace.h"
#include <unsupported/Eigen/MatrixFunctions>

double R2BeliefSpace::StateType::meanNormWeight_  = -1;
double R2BeliefSpace::StateType::covNormWeight_   = -1;
double R2BeliefSpace::StateType::reachDist_   = -1;
// arma::colvec R2BeliefSpace::StateType::normWeights_ = arma::zeros<arma::colvec>(3);

bool R2BeliefSpace::StateType::isReached(ompl::base::State *state, bool relaxedConstraint) const
{

    Eigen::Vector4d stateDiff = this->getMatrixData() - state->as<R2BeliefSpace::StateType>()->getMatrixData();
    double meanNorm = stateDiff.norm();

    double reachConstraint  = reachDist_;

    if(relaxedConstraint)
        reachConstraint *= 4;

    if(meanNorm <= reachConstraint)
    {
        return true;
    }

    return false;
    
}

ompl::base::State* R2BeliefSpace::allocState(void) const
{

    StateType *rstate = new StateType();
    rstate->values = new double[dimension_];

    return rstate;
}

void R2BeliefSpace::copyState(State *destination, const State *source) const
{
    destination->as<StateType>()->setX(source->as<StateType>()->getX());
    destination->as<StateType>()->setXdot(source->as<StateType>()->getXdot());
    destination->as<StateType>()->setY(source->as<StateType>()->getY());
    destination->as<StateType>()->setYdot(source->as<StateType>()->getYdot());
    destination->as<StateType>()->setSigma(source->as<StateType>()->getSigma());
    destination->as<StateType>()->setLambda(source->as<StateType>()->getLambda());
}

void R2BeliefSpace::freeState(State *state) const
{
    RealVectorStateSpace::freeState(state);
}

double R2BeliefSpace::distance(const State* state1, const State *state2) const //wasserstein distance
{

    //returns the wasserstein distance
    double dx = state1->as<StateType>()->getX() - state2->as<StateType>()->getX();
    double dxd = state1->as<StateType>()->getXdot() - state2->as<StateType>()->getXdot();
    double dy = state1->as<StateType>()->getY() - state2->as<StateType>()->getY();
    double dyd = state1->as<StateType>()->getYdot() - state2->as<StateType>()->getYdot();
    Eigen::Matrix4d cov1 = state1->as<StateType>()->getCovariance();
    Eigen::Matrix4d cov2 = state2->as<StateType>()->getCovariance();
    return pow(dx*dx+dy*dy+dxd*dxd+dyd*dyd, 0.5); // + (cov1 + cov2 - 2*(cov2.sqrt()*cov1*cov2.sqrt()).sqrt()).trace(); //TODO - uncomment for wasserstein
}

void R2BeliefSpace::printBeliefState(const State *state)
{
    std::cout<<"----Printing BeliefState----"<<std::endl;
    std::cout<<"State [X, Xd, Y, Yd]: ";
    std::cout<<"["<<state->as<R2BeliefSpace::StateType>()->getX()<<", "<<state->as<R2BeliefSpace::StateType>()->getXdot()<<", "<<state->as<R2BeliefSpace::StateType>()->getY()<<", "<<state->as<R2BeliefSpace::StateType>()->getYdot()<<"]"<<std::endl;
    std::cout<<"Covariance  is" <<std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getCovariance()<<std::endl;
    std::cout<<"Sigma is" << std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getSigma()<<std::endl;
    std::cout<<"Lambda is" << std::endl;
    std::cout<<state->as<R2BeliefSpace::StateType>()->getLambda()<<std::endl;
    std::cout<<"------End BeliefState-------"<<std::endl;
}

//added by roland, for printing purposes
void R2BeliefSpace::copyToReals(std::vector<double> &reals, const State *source) const
{
    reals.clear(); 
    auto cov = source->as<R2BeliefSpace::StateType>()->getCovariance();
    auto state = source->as<R2BeliefSpace::StateType>()->getMean();
    reals.push_back(state[0]);
    reals.push_back(state[1]);
    reals.push_back(state[2]);
    reals.push_back(state[3]);
    reals.push_back(cov(0,0));
    reals.push_back(cov(0,1));
    reals.push_back(cov(0,2));
    reals.push_back(cov(0,3));
    reals.push_back(cov(1,1));
    reals.push_back(cov(1,2));
    reals.push_back(cov(1,3));
    reals.push_back(cov(2,2));
    reals.push_back(cov(2,3));
    reals.push_back(cov(3,3));
}