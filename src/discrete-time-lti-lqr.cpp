
#include <sot-stabilizer/tools/discrete-time-non-alg-riccati-eqn.hh>
#include <sot-stabilizer/controllers/discrete-time-lti-lqr.hh>

#include <iostream>


namespace sotStabilizer
{
namespace controller
{
    DiscreteTimeLTILQR::DiscreteTimeLTILQR(unsigned stateSize, unsigned controlSize)
    {
        stateSize_=stateSize;
        controlSize_=controlSize;
        changedValue_=true;
        computedInput_=false;
        remainingTime_=-1;
        horizon_=10;
        ratioGains_=1;
    }

    unsigned DiscreteTimeLTILQR::getStateSize() const
    {
        return stateSize_;
    }

    unsigned DiscreteTimeLTILQR::getControlSize() const
    {
        return controlSize_;
    }

    stateObservation::Vector DiscreteTimeLTILQR::getControl(int time)
    {
        if (time==time_)
        {

#ifndef NDEBUG
            std::cout<<"Time :"<< time<< std::endl;
#endif // NDEBUG

            if (changedValue_)
            {

                sotStabilizer::tools::discreteTimeNonAlgRiccatiEqn(A_,B_,Q_,R_,Qn_,Pn_,horizon_);
                changedValue_=false;
                stateObservation::Matrix P(Pn_.front());
                lastGain_ = (R_+B_.transpose()*P*B_).inverse()*B_.transpose()*P*A_;
#ifndef NDEBUG
                std::cout<<"A :"<< std::endl;
                std::cout<< A_ <<std::endl;
                std::cout<<"B :"<< std::endl;
                std::cout<< B_.transpose() <<std::endl;
                std::cout<<"Q :"<< std::endl;
                std::cout<< Q_ <<std::endl;
                std::cout<<"R :"<< std::endl;
                std::cout<< R_ <<std::endl;
#endif
            }
            else
            {
//                std::cout<<"Gain :"<< -lastGain_ <<std::endl;
            }


            checkState_(x_);
            u_ = -ratioGains_*lastGain_* x_;

            if (remainingTime_>0)
            {
                --remainingTime_;
                Pn_.popFront();
            }

            computedInput_=true;

#ifndef NDEBUG

            std::cout<< "P " << std::endl;
            std::cout<< P << std::endl;
            std::cout<<"Gain :"<< std::endl;
            std::cout<< -lastGain_ <<std::endl;
            std::cout<<"State :"<< x_.transpose()<< std::endl;
            std::cout<<"control :"<< u_.transpose()<< std::endl;
            std::cout<<std::endl;
#endif


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


    void DiscreteTimeLTILQR::setControlDuration(int t)
    {
        remainingTime_ = t;
    }

    void DiscreteTimeLTILQR::setHorizonLength(int h)
    {
        horizon_ = h;
    }

    void DiscreteTimeLTILQR::setDynamicsMatrices(const stateObservation::Matrix & A,
                                                const stateObservation::Matrix & B)
    {
        checkDynamicsMatrices_(A,B);

        A_=A;
        B_=B;
//        std::cout << "changedValue_ DYNAMICS-----------------------------------"<<std::endl;
        changedValue_=true;
    }

    void DiscreteTimeLTILQR::setCostMatrices ( const stateObservation::Matrix & Q,
                                                const stateObservation::Matrix & R,
                                                const stateObservation::Matrix & Qn)
    {
        checkCostMatrices_(Q,R,Qn);
        Q_=Q;
        R_=R;
        Qn_=Qn;
//        std::cout << "changedValue_ COST-----------------------------------"<<std::endl;
        changedValue_=true;
    }

    void DiscreteTimeLTILQR::setCostMatrices (const stateObservation::Matrix& Q,
                                              const stateObservation::Matrix& R )
    {
        setCostMatrices(Q,R,Q);
    }

    stateObservation::Matrix DiscreteTimeLTILQR::getLastGain() const
    {
        return lastGain_;
    }

    void DiscreteTimeLTILQR::setGains (const stateObservation::Matrix & k)
    {
        lastGain_=k;
    }
}
}
