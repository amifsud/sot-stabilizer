/*
 * Copyright 2014,
 * Mehdi Benallegue
 *
 * CNRS
 *
 * This file is part of dynamic-graph-tutorial.
 * dynamic-graph-tutorial is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 * dynamic-graph-tutorial is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.  You should
 * have received a copy of the GNU Lesser General Public License along
 * with dynamic-graph-tutorial.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <state-observation/tools/definitions.hpp>
#include <sot-stabilizer/controllers/controller-base.hh>


#ifndef DISCRETETIMEFINITEHORIZONLQRHH
#define DISCRETETIMEFINITEHORIZONLQRHH

#define NDEBUG

namespace sotStabilizer
{
namespace controller
{

class DiscreteTimeLTILQR: public ControllerBase
{
public:

    DiscreteTimeLTILQR(unsigned stateSize, unsigned controlSize);

    virtual ~DiscreteTimeLTILQR(){}

    virtual stateObservation::Vector getControl(int time);

    virtual unsigned getStateSize() const ;

    virtual unsigned getControlSize() const ;


    //-1 for infinite
    virtual void setControlDuration(int t);

    virtual void setHorizonLength(int h);

    virtual void setDynamicsMatrices (const stateObservation::Matrix & A,
                                      const stateObservation::Matrix & B);

    virtual void setCostMatrices (const stateObservation::Matrix & Q,
                                  const stateObservation::Matrix & R,
                                  const stateObservation::Matrix & Qn);

    virtual void setCostMatrices (const stateObservation::Matrix & Q,
                                  const stateObservation::Matrix & R );

    virtual void setGains ( const stateObservation::Matrix & k);

    virtual stateObservation::Matrix getLastGain() const ;

    virtual void setRatioGains(const double & r)
    {
        ratioGains_=r;
    }

    virtual double getRatioGains() const
    {
        return ratioGains_;
    }

protected:
    inline bool checkDynamicsMatrices_(const stateObservation::Matrix& A,
                                        const stateObservation::Matrix& B)
    {
        BOOST_ASSERT(A.rows() == getStateSize() && A.cols() == getStateSize()
                                            && "The A matrix size is incorrect");
        BOOST_ASSERT(B.rows() == getStateSize() && B.cols() == getControlSize()
                                            && "The B matrix size is incorrect");
        return (A.rows() == getStateSize() && A.cols() == getStateSize()
                    && B.rows() == getStateSize() && B.cols() == getControlSize());
    }

    inline bool checkCostMatrices_
        (const stateObservation::Matrix& Q,
         const stateObservation::Matrix& R,
         const stateObservation::Matrix& Qn)
    {
        BOOST_ASSERT(Q.rows() == getStateSize() && Q.cols() == getStateSize()
                                            && "The Q matrix size is incorrect");
        BOOST_ASSERT(Qn.rows() == getStateSize() && Qn.cols() == getStateSize()
                                            && "The Qn matrix size is incorrect");
        BOOST_ASSERT(R.rows() == getControlSize() && R.cols() == getControlSize()
                                            && "The R matrix size is incorrect");
        return (Q.rows() == getStateSize() && Q.cols() == getStateSize()
                && Qn.rows() == getStateSize() && Qn.cols() == getStateSize()
                && R.rows() == getControlSize() && R.cols() == getControlSize());
    }



    unsigned stateSize_;
    unsigned controlSize_;

    stateObservation::Matrix Q_,Qn_,R_;
    stateObservation::Matrix A_,B_;
    stateObservation::IndexedMatrixArray Pn_;

    bool computedInput_;
    int remainingTime_;
    unsigned horizon_;
    bool changedValue_;

    stateObservation::Matrix lastGain_;
    double ratioGains_;

    stateObservation::Vector u_;

};
}
}

#endif//DISCRETETIMEFINITEHORIZONLQRHH
