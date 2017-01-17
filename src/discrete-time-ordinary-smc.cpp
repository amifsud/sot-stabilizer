
#include <sot-stabilizer/tools/discrete-time-non-alg-riccati-eqn.hh>
#include <sot-stabilizer/controllers/discrete-time-ordinary-smc.hh>

#include <iostream>


namespace sotStabilizer
{
namespace controller
{
    DiscreteTimeOrdinarySMC::DiscreteTimeOrdinarySMC(unsigned stateSize, unsigned controlSize):
            DiscreteTimeSMC(stateSize,controlSize)
    {
        dt_ = 0.005;

        alpha_ = 5;

        underActuatedSize_ = stateSize_-controlSize_;
        lambdaa_.resize(controlSize_,controlSize_);
        lambda0_.resize(controlSize_,underActuatedSize_);
        alpha0_.resize(controlSize_,underActuatedSize_ );

        xDerivative_.resize(stateSize_);
        xDerivativeRef_.resize(stateSize_);
    }

    void DiscreteTimeOrdinarySMC::computeSurface()
    {
        stateObservation::Vector dea, ea, de0, e0;
        dea = xDerivative_.segment(0,controlSize_) - xDerivativeRef_.segment(0,controlSize_);
        ea = x_.segment(0,controlSize_) - xRef_.segment(0,controlSize_);
        de0 = xDerivative_.segment(controlSize_,underActuatedSize_) - xDerivativeRef_.segment(controlSize_,underActuatedSize_);
        e0 = x_.segment(controlSize_,underActuatedSize_) - xRef_.segment(controlSize_,underActuatedSize_);
        s_ = dea + lambdaa_*ea + alpha0_*de0 + lambda0_*e0;
    }

    void DiscreteTimeOrdinarySMC::computeControl()
    {
        u_ = - alpha_*sign(s_);
    }
}
}
