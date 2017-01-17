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
#include <sot-stabilizer/controllers/discrete-time-smc.hh>


#ifndef DISCRETETIMEORDINARYSMC
#define DISCRETETIMEORDINARYSMC

#define NDEBUG

namespace sotStabilizer
{
namespace controller
{
    class DiscreteTimeOrdinarySMC: public DiscreteTimeSMC
    {
    public:

        DiscreteTimeOrdinarySMC(unsigned stateSize, unsigned controlSize);

        virtual ~DiscreteTimeOrdinarySMC(){}

        void setStateDerivative(stateObservation::Vector dx)
        {
            xDerivative_ = dx;
        }

        void setStateDerivativeRef(stateObservation::Vector dx)
        {
            xDerivativeRef_ = dx;
        }

        void computeSurface();
        void computeControl();

    protected:

        stateObservation::Vector xDerivative_;
        stateObservation::Vector xDerivativeRef_;

        double alpha_;

        unsigned underActuatedSize_;
        stateObservation::Matrix lambdaa_, lambda0_, alpha0_;
    };
}
}

#endif//DISCRETETIMEORDUNARYSMC
