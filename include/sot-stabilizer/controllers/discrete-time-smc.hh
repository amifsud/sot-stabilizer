/*
 * Copyright 2017,
 * Alexis Mifsud
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


#ifndef DISCRETETIMESMC
#define DISCRETETIMESMC

#define NDEBUG

namespace sotStabilizer
{
namespace controller
{
    class DiscreteTimeSMC: public ControllerBase
    {
    public:

        DiscreteTimeSMC(unsigned stateSize, unsigned controlSize);
        virtual ~DiscreteTimeSMC(){}

        unsigned getStateSize() const
        {
            return stateSize_;
        }

        unsigned getControlSize() const
        {
            return controlSize_;
        }

        void setState(stateObservation::Vector x)
        {
            x_=x;
        }

        void setStateRef(stateObservation::Vector x)
        {
            xRef_=x;
        }

        stateObservation::Vector & sign(stateObservation::Vector & v)
        {
            opt_.sign.resize(v.size());
            for(int i=0; i<v.size(); ++i)
            {
                opt_.sign[i]=int(v[i]/std::abs(v[i]));
            }
            return opt_.sign;
        }

        double & exp(double x, unsigned y)
        {
            double xOut(x);
            for(int i=0; i<y; ++i) xOut*=x;
            return xOut;
        }

        virtual stateObservation::Vector & computeSurface() = 0;
        virtual stateObservation::Vector & computeControl() = 0;
        stateObservation::Vector getControl(int time);


    protected:

        double dt_;

        unsigned stateSize_;
        unsigned controlSize_;

        bool computedInput_;

        stateObservation::Vector x_;
        stateObservation::Vector xRef_;
        stateObservation::Vector s_;
        stateObservation::Vector u_;

        class optimization
        {
        public:
            stateObservation::Vector sign;
        } opt_;
    };
}
}

#endif//DISCRETETIMESMC
