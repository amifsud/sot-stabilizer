
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

        alpha_ = 18;
        order_ = 1;
        lambda_=1;

        xDerivative_.resize(stateSize_);
    }

    void DiscreteTimeOrdinarySMC::computeSurface()
    {
        s_ = exp(lambda_,order_)*(x_-xRef_)+xDerivative_;
    }

    void DiscreteTimeOrdinarySMC::computeControl()
    {
        u_ = - alpha_*sign(s_);
    }
}
}
