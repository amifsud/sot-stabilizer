//
// Copyright (c) 2014,
// Alexis Mifsud
//
// CNRS
//
// This file is part of sot-dynamic.
// sot-dynamic is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
// sot-dynamic is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.  You should
// have received a copy of the GNU Lesser General Public License along
// with sot-dynamic.  If not, see <http://www.gnu.org/licenses/>.
//

#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/command-bind.h>

#include <state-observation/tools/definitions.hpp>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/resolve-momentum-control.hh>
#include <stdexcept>
#include "boost/date_time/posix_time/posix_time.hpp"

namespace sotStabilizer
{
  using dynamicgraph::sot::TaskAbstract;
  using dynamicgraph::Signal;
  using dynamicgraph::SignalPtr;
  using dynamicgraph::SignalTimeDependent;
  using dynamicgraph::Vector;
  using dynamicgraph::Matrix;
  using dynamicgraph::Entity;
  using dynamicgraph::sot::VectorMultiBound;
  using dynamicgraph::command::makeCommandVoid0;
  using dynamicgraph::command::docCommandVoid0;
  using dynamicgraph::command::docDirectSetter;
  using dynamicgraph::command::makeDirectSetter;
  using dynamicgraph::command::docDirectGetter;
  using dynamicgraph::command::makeDirectGetter;
  using dynamicgraph::sot::MatrixHomogeneous;
  using dynamicgraph::sot::MatrixRotation;
  using dynamicgraph::sot::VectorUTheta;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (ResolveMomentumControl, "ResolveMomentumControl");

/// Dynamic balance stabilizer
///
/// This task takes as input the following signals
/// and provides as output the following signals
/// \li taskSOUT, the desired task,
/// \li jacobianSOUT, the jacobian of the desired task
  ResolveMomentumControl::ResolveMomentumControl(const std::string& inName) :
    TaskAbstract(inName),
    on_(true), dt_(0.005)
  {

    // Set dependencies
        // taskSOUT dependencies
    //taskSOUT.addDependency (*); for all input signals

        // jacobianSOUT dependencies
    //jacobianSOUT.addDependency (*); for all jacobian input signals

    // Setting methods for output signals
    taskSOUT.setFunction (boost::bind(&ResolveMomentumControl::computeTask,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&ResolveMomentumControl::computeJacobian,this,_1,_2));

  }

 /// Compute the task
  VectorMultiBound&  ResolveMomentumControl::computeTask(VectorMultiBound& task, const int& time)
  {
    return task;
  }

  Matrix& ResolveMomentumControl::computeJacobian(Matrix& jacobian, const int& time)
  {
    return jacobian;
  }

} // namespace sotStabilizer
