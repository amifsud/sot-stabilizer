
#include <sot-stabilizer/tools/discrete-time-non-alg-riccati-eqn.hh>
#include <sot-stabilizer/controllers/discrete-time-smc.hh>

#include <iostream>


namespace sotStabilizer
{
namespace controller
{
    DiscreteTimeSMC::DiscreteTimeSMC(unsigned stateSize, unsigned controlSize)
    {
        stateSize_=stateSize;
        controlSize_=controlSize;

        u_.resize(controlSize_);
        x_.resize(stateSize_);
        xRef_.resize(stateSize_);
        s_.resize(stateSize_);
    }

    stateObservation::Vector DiscreteTimeSMC::getControl(int time)
    {
        if (time==time_)
        {

#ifndef NDEBUG
            std::cout<<"Time :"<< time<< std::endl;
#endif // NDEBUG*

            computeSurface();
            computeControl();

            time_ = time_+1;
        }
        else
        {
            BOOST_ASSERT(time_==time-1 &&
             "The requested control time should be current value or next value.");

            BOOST_ASSERT( computedInput_ &&
             "Input is not initalized, try requesting for current time.");

        }
        return u_;
    }

}
}
