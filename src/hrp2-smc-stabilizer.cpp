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

#include <sot-stabilizer/hrp2-smc-stabilizer.hh>
#include <stdexcept>

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

  const unsigned stateSize_=6;
  const unsigned controlSize_=3;
  const unsigned taskSize_=3;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2SMCStabilizer, "HRP2SMCStabilizer");

/// Dynamic balance stabilizer
///
/// This task takes as input four signals
/// \li comSIN, the position of the center of mass (COM)
/// \li comRefSIN, the the desired position of the center of mass,
/// \li zmpSIN, the position of the center of pressure (ZMP),
/// \li zmpDesSIN, the desired position of the center of pressure,
/// \li jacobianSIN, the jacobian of the center of mass,
/// \li comdotSIN, reference velocity of the center of mass,
/// and provides as output two signals
/// \li taskSOUT, the desired time derivative of the center of mass,
/// \li jacobianSOUT, the jacobian of the center of mass
  HRP2SMCStabilizer::HRP2SMCStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::com"),
    comDotSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comDot"),
    comRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comRef"),
    comDotRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comDotRef"),
    jacobianComSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::Jcom"),
    stateSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::state"),
    stateRefSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::stateRef"),
    stateErrorSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::stateError"),
    controlSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::control"),
    controlGainSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(double)::controlGain"),
    dt_ (.005), on_ (false),
    controller_(stateSize_,controlSize_)
  {

    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comDotSIN_);
    signalRegistration (comRefSIN_);
    signalRegistration (comDotRefSIN_);
    signalRegistration (jacobianComSIN_);
    signalRegistration (controlGainSIN_);

    signalRegistration (stateSOUT_);
    signalRegistration (stateRefSOUT_);
    signalRegistration (stateErrorSOUT_);
    signalRegistration (controlSOUT_);

    // Set dependencies
        // taskSOUT dependencies
    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comDotSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (comDotRefSIN_);
    taskSOUT.addDependency (jacobianComSIN_);
        // jacobianSOUT dependencies
    jacobianSOUT.addDependency (jacobianComSIN_);

    controlSOUT_.addDependency (taskSOUT);

    // Settinf methods for output signals
    taskSOUT.setFunction (boost::bind(&HRP2SMCStabilizer::computeControlFeedback,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2SMCStabilizer::computeJacobian,this,_1,_2));

    controlSOUT_.setFunction (boost::bind(&HRP2SMCStabilizer::getControl,this,_1,_2));

    std::string docstring;
    docstring =
      "\n"
      "    Set sampling time period task\n"
      "\n"
      "      input:\n"
      "        a floating point number\n"
      "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2SMCStabilizer, double>
               (*this, &HRP2SMCStabilizer::setTimePeriod, docstring));

    docstring =
      "\n"
      "    Get sampling time period task\n"
      "\n"
      "      return:\n"
      "        a floating point number\n"
      "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2SMCStabilizer, double>
               (*this, &HRP2SMCStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2SMCStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2SMCStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));


    stateObservation::Vector vect;

    vect.resize(3);
    vect <<  0.0,
            0,
            0.80771;
    comSIN_.setConstant(convertVector<dynamicgraph::Vector>(vect));

    vect.setZero();
    comDotSIN_.setConstant(convertVector<dynamicgraph::Vector>(vect));

    preTask_.resize(taskSize_);
    preTask_.setZero();

    uk_.resize(controlSize_);
    xk_.resize(stateSize_);
    xkRef_.resize(stateSize_);
    dxk_.resize(stateSize_);

  }

  Vector& HRP2SMCStabilizer::getControl(Vector& control, const int& time)
  {
      taskSOUT.access(time);
      control=controlSOUT_.access (time);
      return control;
  }

 /// Compute the control law
  VectorMultiBound&
  HRP2SMCStabilizer::computeControlFeedback(VectorMultiBound& task,
      const int& time)
  {

    // State
    const stateObservation::Vector & com = convertVector<stateObservation::Vector>(comSIN_.access(time));
    const stateObservation::Vector & comDot = convertVector<stateObservation::Vector>(comDotSIN_ (time));

    // State Reference
    const stateObservation::Vector & comRef = convertVector<stateObservation::Vector>(comRefSIN_ (time));
    const stateObservation::Vector & comDotRef = convertVector<stateObservation::Vector>(comDotRefSIN_ (time));

    // Control gain
    const double& gain = controlGainSIN_.access (time);

    // State reconstruction;
    xk_.resize(stateSize_);
    xk_ <<  com,
            comDot;
    stateSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xk_));

    // Reference reconstruction
    xkRef_.resize(stateSize_);
    xkRef_ <<   comRef,
                comDotRef;
    stateRefSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkRef_));

    // State error
    dxk_=xk_-xkRef_;
    stateErrorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(dxk_));

    /// Computing control
    preTask_ <<  -gain*dxk_;

    /// Post treatments

    controlSOUT_.setConstant (convertVector<dynamicgraph::Vector>(uk_));
    controlSOUT_.setTime (time);

    /// Computing task
    task.resize (taskSize_);
    for (int i=0;i<taskSize_;i++)
    {
        task [i].setSingleBound (preTask_(i));
    }

    return task;
  }

  Matrix& HRP2SMCStabilizer::computeJacobian(Matrix& jacobian, const int& time)
  {
    const stateObservation::Matrix & jacobianCom=convertMatrix<stateObservation::Matrix>(jacobianComSIN_(time));

    stateObservation::Matrix preJacobian;
    preJacobian.resize(jacobianCom.rows(),jacobianCom.cols());
    preJacobian.block(0,0,jacobianCom.rows(),jacobianCom.cols())= jacobianCom;

    jacobian = convertMatrix<dynamicgraph::Matrix>(preJacobian);
    return jacobian;
  }

} // namespace sotStabilizer
