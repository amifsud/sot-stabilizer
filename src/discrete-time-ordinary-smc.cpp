
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

        alpha_.resize(controlSize_);
        alpha_ << 0.5,
                  0.5,
                  0.5,
                  0.5,
                  0.5,
                  0.5;
        alpha1_ = 5;

        underActuatedSize_ = stateSize_-controlSize_;

        xDerivative_.resize(stateSize_);
        xDerivativeRef_.resize(stateSize_);

        ns0_=0.0;
        du1_.resize(controlSize_);
        u1_.resize(controlSize_);
    }

    void DiscreteTimeOrdinarySMC::computeSurface()
    {
        stateObservation::Vector dea, ea, de0, e0;
        dea = xDerivative_.segment(0,controlSize_) - xDerivativeRef_.segment(0,controlSize_);
        ea = x_.segment(0,controlSize_) - xRef_.segment(0,controlSize_);
        de0 = xDerivative_.segment(controlSize_,underActuatedSize_) - xDerivativeRef_.segment(controlSize_,underActuatedSize_);
        e0 = x_.segment(controlSize_,underActuatedSize_) - xRef_.segment(controlSize_,underActuatedSize_);
        s_ = dea + lambdaa_*ea + alpha0_*de0 + lambda0_*e0;

        std::cout << "x_=" << x_.segment(0,controlSize_).transpose() << std::endl;
        std::cout << "xRef_=" << xRef_.segment(0,controlSize_).transpose() << std::endl;
        std::cout << "ea=" << ea.transpose() << std::endl;
        std::cout << "dea=" << dea.transpose() << std::endl;
        std::cout << "s_=" << s_.transpose() << std::endl;
    }

    void DiscreteTimeOrdinarySMC::computeControl()
    {
//        if(ns0_==0.0) ns0_= s_.norm();

//        u1_+=du1_*dt_;

//        if(s_.norm()<ns0_)
//        {
        for(int i=0; i<controlSize_;++i) u_[i]= - alpha_[i]*std::sqrt(s_.norm())*sign(s_)[i];
//        }
//        else
//        {
//            u_ = - alpha_*std::sqrt(ns0_)*sign(s_) + u1_;
//        }

//        if(u_.norm()>1)
//        {
//            du1_=-u_;
//        }
//        else
//        {
//            du1_ = - alpha1_*sign(s_);
//        }

    }
}
}
