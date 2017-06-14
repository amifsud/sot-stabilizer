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

#include <sot-stabilizer/hrp2-lqr-twoDof-coupled-stabilizer.hh>
#include <stdexcept>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <iostream>

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

  double HRP2LQRTwoDofCoupledStabilizer::constm_ = 56.8;
  double HRP2LQRTwoDofCoupledStabilizer::constcomHeight_ = 0.807;
  double HRP2LQRTwoDofCoupledStabilizer::conststepLength_ = 0.19;

  const unsigned stateSize_=14;
  const unsigned controlSize_=5;
  const unsigned taskSize_=7;

  DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2LQRTwoDofCoupledStabilizer, "HRP2LQRTwoDofCoupledStabilizer");

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
  HRP2LQRTwoDofCoupledStabilizer::HRP2LQRTwoDofCoupledStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::com"),
    waistHomoSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::waistHomo"),
    flexOriVectSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexOriVect"),
    comDotSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comDot"),
    waistAngVelSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistAngVel"),
    flexAngVelVectSIN_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexAngVelVect"),
    comBiasSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comBias"),
    tflexSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::tflex"),
    dtflexSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::dtflex"),
    comRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comRef"),
    perturbationVelSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::perturbationVel"),
    perturbationAccSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::perturbationAcc"),
    waistOriRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistOriRef"),
    flexOriRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexOriRef"),
    comDotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comDotRef"),
    waistVelRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistVelRef"),
    flexAngVelRefSIN_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::flexAngVelRef"),
    jacobianComSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    jacobianWaistSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jwaist"),
    jacobianChestSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::Jchest"),
    controlGainSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(double)::controlGain"),
    inertiaSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::inertia"),
    angularmomentumSIN(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::angularmomentum"),
    stateSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::state"),
    stateWorldSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateWorld"),
    stateErrorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateError"),
    stateRefSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateRef"),
    stateSimulationSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateSimulation"),
    statePredictionSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::statePrediction"),
    stateExtendedSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector):stateExtended"),
    stateModelErrorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector):stateModelError"),
    errorSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::error"),
    controlSOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::control"),
    gainSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::gain"),
    nbSupportSIN_ (0x0,"HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(unsigned)::nbSupport"),
    supportPos1SIN_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::supportPos1"),
    supportPos2SIN_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::supportPos2"),
    AmatrixSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::Amatrix"),
    BmatrixSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::Bmatrix"),
    inertiaSOUT(0x0 , "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::inertiaOut"), 
    waistAngAccSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::waistAngAcc"),
        comddotRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::comddotRef"),
    zmpRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::zmpRef"),
    stateFlexDDotSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::stateFlexDDot"),
    zmpRefSOUT_("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::zmpRefOUT"),
    energySOUT_ ("HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::energy"),
    computationTimeSOUT (NULL,"HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::computationTime"),
    dt_ (.005), on_ (false),
    supportPos1_(3), supportPos2_(3),
    fixedGains_(true), zmpMode_(true), computed_(false),
    zmp_ (3), xpredicted_(stateSize_),
    controller_(stateSize_,controlSize_),
    A_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    B_(stateObservation::Matrix::Zero(stateSize_,controlSize_)),
    Q_(stateObservation::Matrix::Zero(stateSize_,stateSize_)),
    R_(stateObservation::Matrix::Zero(controlSize_,controlSize_)),
    I_(3,3), constantInertia_(false), xSimu_(stateSize_)
  {

    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (waistHomoSIN_);
    signalRegistration (flexOriVectSIN_);
    signalRegistration (comDotSIN_);
    signalRegistration (waistAngVelSIN_);
    signalRegistration (flexAngVelVectSIN_);
    signalRegistration (tflexSIN_);
    signalRegistration (dtflexSIN_);
    signalRegistration (comBiasSIN_);
    signalRegistration (comRefSIN_);
    signalRegistration (perturbationVelSIN_);
    signalRegistration (perturbationAccSIN_);
    signalRegistration (waistOriRefSIN_);
    signalRegistration (flexOriRefSIN_);
    signalRegistration (comDotRefSIN_);
    signalRegistration (waistVelRefSIN_);
    signalRegistration (flexAngVelRefSIN_);
    signalRegistration (jacobianComSIN_);
    signalRegistration (jacobianWaistSIN_);
    signalRegistration (jacobianChestSIN_);
    signalRegistration (controlGainSIN_);
    signalRegistration (inertiaSIN);
    signalRegistration (angularmomentumSIN);
    signalRegistration (stateSOUT_);
    signalRegistration (stateWorldSOUT_);
    signalRegistration (stateRefSOUT_);
    signalRegistration (stateSimulationSOUT_);
    signalRegistration (statePredictionSOUT_);
    signalRegistration (stateErrorSOUT_);
    signalRegistration (stateExtendedSOUT_);
    signalRegistration (stateModelErrorSOUT_);
    signalRegistration (errorSOUT_);
    signalRegistration (controlSOUT_);
    signalRegistration (gainSOUT);
    signalRegistration (nbSupportSIN_ << supportPos1SIN_ << supportPos2SIN_);
    signalRegistration (AmatrixSOUT);
    signalRegistration (BmatrixSOUT);
    signalRegistration (inertiaSOUT);
    signalRegistration (waistAngAccSIN_);
    signalRegistration (comddotRefSIN_);
    signalRegistration (zmpRefSIN_);
    signalRegistration (stateFlexDDotSIN_);
    signalRegistration (zmpRefSOUT_);
    signalRegistration (energySOUT_);
    signalRegistration (computationTimeSOUT);

    // Set dependencies
        // taskSOUT dependencies
    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (waistHomoSIN_);
    taskSOUT.addDependency (flexOriVectSIN_);
    taskSOUT.addDependency (comDotSIN_);
    taskSOUT.addDependency (waistAngVelSIN_);
    taskSOUT.addDependency (flexAngVelVectSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (waistOriRefSIN_);
    taskSOUT.addDependency (flexOriRefSIN_);
    taskSOUT.addDependency (comDotRefSIN_);
    taskSOUT.addDependency (waistVelRefSIN_);
    taskSOUT.addDependency (flexAngVelRefSIN_);
    taskSOUT.addDependency (jacobianComSIN_);
    taskSOUT.addDependency (jacobianWaistSIN_);
    taskSOUT.addDependency (controlGainSIN_);
    taskSOUT.addDependency (inertiaSIN);
        // jacobianSOUT dependencies
    jacobianSOUT.addDependency (jacobianComSIN_);
    jacobianSOUT.addDependency (jacobianWaistSIN_);

    controlSOUT_.addDependency (taskSOUT);

    if (zmpMode_){
      taskSOUT.addDependency(zmpRefSIN_);
    }else{
      taskSOUT.addDependency(comddotRefSIN_);
    }

    // Settinf methods for output signals
    taskSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::computeJacobian,this,_1,_2));

    controlSOUT_.setFunction (boost::bind(&HRP2LQRTwoDofCoupledStabilizer::getControl,this,_1,_2));

    std::string docstring;
    docstring =
      "\n"
      "    Set sampling time period task\n"
      "\n"
      "      input:\n"
      "        a floating point number\n"
      "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, double>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setTimePeriod, docstring));

    docstring =
      "\n"
      "    Set if yes or no the gains are fixed\n"
      "\n"
      "      input:\n"
      "        boolean\n"
      "\n";
    addCommand("setFixedGains",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, bool>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setFixedGains, docstring));


    docstring =
      "\n"
      "    Get sampling time period task\n"
      "\n"
      "      return:\n"
      "        a floating point number\n"
      "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, double>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2LQRTwoDofCoupledStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("stop",
                makeCommandVoid0 (*this, &HRP2LQRTwoDofCoupledStabilizer::stop,
                                  docCommandVoid0 ("Stop stabilizer")));


    docstring =
      "\n"
      "    Set the cost Matrix for Input\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setInputCost",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setInputCost, docstring));

    docstring =
      "\n"
      "    Set the cost Matrix for state (single support)\n"
      "\n"
      "      input:\n"
      "        Matrix\n"
      "\n";
    addCommand("setStateCost",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setStateCost, docstring));


    docstring =
      "\n"
      "    Set if we use zmp reference or CoM acceleration\n"
      "\n"
      "      input:\n"
      "        Bool\n"
      "\n";
    addCommand("setZmpMode",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, bool>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setZMPMode, docstring));


    docstring =
      "\n"
      "    Get the cost Matrix for Input\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getInputCost",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getInputCost, docstring));

    docstring =
      "\n"
      "    Get the cost Matrix for state (simple support)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getStateCost",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getStateCost, docstring));

    docstring =
      "\n"
      "    Get the last Gain matrix for simple support (x axis)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastGains",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getLastGain, docstring));

    docstring =
      "\n"
      "    Get the last floored Gain matrix for simple support (x axis)\n"
      "\n"
      "      output:\n"
      "        Matrix\n"
      "\n";
    addCommand("getLastFloorGains",
               new dynamicgraph::command::Getter<HRP2LQRTwoDofCoupledStabilizer, Matrix>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::getLastFloorGain, docstring));

    docstring =
      "\n"
      "    Set the horizon for the LQR\n"
      "\n"
      "      input:\n"
      "        int\n"
      "\n";
    addCommand("setHorizon",
               new dynamicgraph::command::Setter<HRP2LQRTwoDofCoupledStabilizer, int>
               (*this, &HRP2LQRTwoDofCoupledStabilizer::setHorizon, docstring));



    docstring  =
            "\n"
            "    Sets the angular stifness of the flexibility \n"
            "\n";

    addCommand(std::string("setKth"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setKth ,docstring));

    docstring  =
            "\n"
            "    Sets the angular damping of the flexibility \n"
            "\n";

    addCommand(std::string("setKdth"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setKdth ,docstring));

    docstring  =
            "\n"
            "    Sets the angular stifness of the flexibility \n"
            "\n";

    addCommand(std::string("setkts"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,double>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setkts ,docstring));

    docstring  =
            "\n"
            "    Sets the angular damping of the flexibility \n"
            "\n";

    addCommand(std::string("setktd"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,double>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setktd ,docstring));

    docstring  =
            "\n"
            "    Sets the linear stifness of the flexibility \n"
            "\n";

    addCommand(std::string("setkfs"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,double>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setkfs ,docstring));

    docstring  =
            "\n"
            "    Sets the angular damping of the flexibility \n"
            "\n";

    addCommand(std::string("setkfd"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,double>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setkfd ,docstring));

    docstring  =
            "\n"
            "    Sets the inertia \n"
            "\n";

    addCommand(std::string("setInertia"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setInertia ,docstring));

    docstring  =
            "\n"
            "    Sets gains \n"
            "\n";

    addCommand(std::string("setGains"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,dynamicgraph::Matrix>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::setGains ,docstring));

    docstring  =
            "\n"
            "    constnant inertia \n"
            "\n";

    addCommand(std::string("constantInertia"),
               new ::dynamicgraph::command::Setter <HRP2LQRTwoDofCoupledStabilizer,bool>
                (*this, & HRP2LQRTwoDofCoupledStabilizer::constantInertia ,docstring));

    Vector rfconf(6);
    rfconf.setZero();
    Vector lfconf(6);
    lfconf.setZero();
    rfconf(0) = 0.009490463094;
    rfconf(1) = -0.095000000000;
    lfconf(0) = 0.009490463094;
    lfconf(1) = 0.095000000000;

    supportPos1SIN_.setConstant (lfconf);
    supportPos1SIN_.setTime (0);
    supportPos1_=lfconf;

    supportPos2SIN_.setConstant (rfconf);
    supportPos2SIN_.setTime (0);
    supportPos2_=rfconf;

    nbSupportSIN_.setConstant (2);
    nbSupportSIN_.setTime (0);

    stateObservation::Vector com;
    com.resize(3);
    com <<  0.0,
            0,
            0.80771;
    comSIN_.setConstant(convertVector<dynamicgraph::Vector>(com));
    com.setZero();
    comBiasSIN_.setConstant(convertVector<dynamicgraph::Vector>(com));

    stateObservation::Matrix4 homoWaist;
    homoWaist <<      0.99998573432883131, -0.0053403256847235764, 0.00010981989355530105, -1.651929567364003e-05,
                      0.0053403915800877009, 0.99998555471196726, -0.00060875707006170711, 0.0008733516988761506,
                      -0.00010656734615829933, 0.00060933486696839291, 0.99999980867719196, 0.64869730032049466,
                      0.0, 0.0, 0.0, 1.0;
    waistHomoSIN_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoWaist));

    stateObservation::Matrix inertia;
    inertia.resize(6,6);
    inertia <<  56.867992000000001, 1.082595017423981e-17, -1.0227922838095677e-19, 0.012404467020412754, 9.0440811406882826, -0.087341805362338931,
                2.1471441204328374e-17, 56.867991999999994, -6.6915602833089727e-20, -9.0441155822106154, 0.012400591461492968, 0.77639199232843892,
                -1.6877131474041934e-19, 7.0557844506283218e-19, 56.867992000000001, 0.086277191499897876, -0.77651101919284393, -6.6174449004242214e-22,
                0.012404467020412752, -9.0441155822106154, 0.086277191499897876, 9.5962629755728397, -0.0033323767643302234, 0.15070837823024946,
                9.0440811406882808, 0.012400591461492966, -0.77651101919284393, -0.0033323567633330438, 8.3906225580324634, -0.038677900732410085,
                -0.087341805362338931, 0.77639199232843892, 6.6174449004242214e-22, 0.15070833849597523, -0.03867792067753683, 1.7394745168929315;
    inertiaSIN.setConstant(convertMatrix<dynamicgraph::Matrix>(inertia));


    Vector vect;
    stateObservation::Vector vec;
    vec.resize(2); vec.setZero();
    vect.resize(3); vect.setZero();
    comDotSIN_.setConstant(vect);
    flexOriVectSIN_.setConstant(vect);
    flexAngVelVectSIN_.setConstant(vect);
    tflexSIN_.setConstant(vect);
    dtflexSIN_.setConstant(vect);
    angularmomentumSIN.setConstant(vect);
    vect.resize(6); vect.setZero();
    waistAngVelSIN_.setConstant(vect);
    waistAngVelSIN_.setTime(0);

    stateObservation::Vector Qvec;
    stateObservation::Matrix Qglob;
    Qglob.resize(stateSize_,stateSize_);
    Qvec.resize(stateSize_);
    Qglob.setIdentity();
    Qglob.noalias()=0.00001*Qglob;
    Qvec    <<  1,     // com
                1,
                1,
                1,     // ori waist
                1,
                1,     // ori flex
                1,
                1,     // dcom
                1,
                1,
                1,     // d ori waist
                1,
                1,     // d ori flex
                1;
    Q_ = Qvec.asDiagonal()*Qglob;

    stateObservation::Vector Rvec;
    stateObservation::Matrix Rglob;
    Rglob.resize(controlSize_,controlSize_);
    Rvec.resize(controlSize_);
    Rglob.setIdentity();
    Rglob.noalias()=1*Rglob;
    Rvec    <<  1,     // dd com
                1,
                1,
                1,     // dd ori waist
                1;

    Q_ = Qvec.asDiagonal()*Qglob;
    R_ = Rvec.asDiagonal()*Rglob;

    zmp_.setZero ();

    int horizon = 4000;

    nbSupport_=2;

    double h2;
    h2=(convertVector<stateObservation::Vector>(supportPos1_).block(0,0,2,1)-convertVector<stateObservation::Vector>(supportPos2_).block(0,0,2,1)).squaredNorm();

    Kth_.resize(3,3);
    Kdth_.resize(3,3);

    kts_=60;
    ktd_=6.5;
    kfs_=600;
    kfd_=65;

    Kth_ <<   0.5*h2*kfs_,0,0,
              0,2*kts_,0,
              0,0,0.5*h2*kfs_;
    Kdth_ <<  0.5*h2*kfd_,0,0,
              0,2*ktd_,0,
              0,0,0.5*h2*kfd_;

    controller_.setHorizonLength(horizon);
    computeDynamicsMatrix(com,Kth_,Kdth_,0);
    controller_.setDynamicsMatrices(A_, B_);
    controller_.setCostMatrices(Q_,R_);

    Vector v(stateSize_);
    v.setZero();
    stateWorldSOUT_.setConstant(v);

    preTask_.resize(controlSize_);
    preTask_.setZero();

    hrp2Mass_ = constm_;

    dynamicgraph::Matrix::Matrix gains;
    gains.resize(controlSize_,stateSize_);
    gains.setZero();
    gainSOUT.setConstant (gains);
    AmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(A_));
    BmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(B_));

    dynamicgraph::Vector zero(6); zero.setZero();
    controlSOUT_.setConstant(zero);

    vect.resize(4);
    energySOUT_.setConstant(vect);

    vect.resize(5);
    vect.setZero();
    perturbationVelSIN_.setConstant(vect);
    perturbationAccSIN_.setConstant(vect);

    I_ <<   8.15831,-0.00380455,0.236677,
            -0.00380453,6.94757,-0.0465754,
            0.236677,-0.0465754,1.73429;

    xSimu_.setZero();

    vect.resize(2);
    vect.setZero();
    computationTimeSOUT.setConstant(vect);

  }

  Vector& HRP2LQRTwoDofCoupledStabilizer::getControl(Vector& control, const int& time)
  {
      taskSOUT.access(time);
      control=controlSOUT_.access (time);
      return control;
  }



 /// Compute the control law
  VectorMultiBound&
  HRP2LQRTwoDofCoupledStabilizer::computeControlFeedback(VectorMultiBound& task,
      const int& time)
  {

    // CoM bias
    const stateObservation::Vector & comBias = convertVector<stateObservation::Vector>(comBiasSIN_.access(time));

    // State
    const stateObservation::Vector & com = convertVector<stateObservation::Vector>(comSIN_.access(time))+comBias;
    const Matrix4& waistHomo = convertMatrix<stateObservation::Matrix>(waistHomoSIN_ (time));
    const stateObservation::Vector & flexOriVect = convertVector<stateObservation::Vector>(flexOriVectSIN_.access(time));
    const stateObservation::Vector & comDot = convertVector<stateObservation::Vector>(comDotSIN_ (time));
    const stateObservation::Vector waistAngVelIn=convertVector<stateObservation::Vector>(waistAngVelSIN_ (time));
    const stateObservation::Vector & waistAngVel = waistAngVelIn.block(3,0,3,1);
    const stateObservation::Vector & flexAngVelVect = convertVector<stateObservation::Vector>(flexAngVelVectSIN_.access(time));

    // Translational part of the flexibility
    const stateObservation::Vector & tflex = convertVector<stateObservation::Vector>(tflexSIN_.access(time));
    const stateObservation::Vector & dtflex = convertVector<stateObservation::Vector>(dtflexSIN_.access(time));

    // State Reference
        // References of velocities and acceleration are equal to zero
    const stateObservation::Vector & comRef = convertVector<stateObservation::Vector>(comRefSIN_ (time));//-comBias;
    const stateObservation::Vector & perturbationVel = convertVector<stateObservation::Vector>(perturbationVelSIN_ (time));
    const stateObservation::Vector & perturbationAcc = convertVector<stateObservation::Vector>(perturbationAccSIN_ (time));

    const stateObservation::Vector & waistOriRef = convertVector<stateObservation::Vector>(waistOriRefSIN_.access(time));
    const stateObservation::Vector & flexOriRef = convertVector<stateObservation::Vector>(flexOriRefSIN_.access(time));
    const stateObservation::Vector & comDotRef = convertVector<stateObservation::Vector>(comDotRefSIN_ (time));
    const stateObservation::Vector & waistAngVelRef = convertVector<stateObservation::Vector>(waistVelRefSIN_ (time));
    const stateObservation::Vector & flexAngVelRef = convertVector<stateObservation::Vector>(flexAngVelRefSIN_.access(time));

    // For energy
    const stateObservation::Vector & angularmomentum = convertVector<stateObservation::Vector>(angularmomentumSIN.access(time));

    // Determination of the number of support
    unsigned int nbSupport=nbSupportSIN_.access(time);
    if (!on_) nbSupport=0;
    supportPos1_=supportPos1SIN_.access(time);
    supportPos2_=supportPos2SIN_.access(time);


    // Control gain
    const double& gain = controlGainSIN_.access (time);

    // Waist orientation
    Matrix3 waistOri=waistHomo.block(0,0,3,3);
    stateObservation::Vector3 waistOriVect;
    waistOriVect=kine::rotationMatrixToRotationVector(waistOri);

    // flex orientation
    Matrix3 flexOri=kine::rotationVectorToRotationMatrix(flexOriVect);

    // State reconstruction
    stateObservation::Vector xk;
    xk.resize(stateSize_);
    xk <<   com,
            (waistOriVect).block(0,0,2,1),
            (flexOriVect).block(0,0,2,1),
            comDot,
            (waistAngVel).block(0,0,2,1),
            (flexAngVelVect).block(0,0,2,1);
    stateSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xk));

    // Reference reconstruction
    stateObservation::Vector xkRef;
    xkRef.resize(stateSize_);
    xkRef <<   comRef,
               waistOriRef.block(0,0,2,1),
               flexOriRef.block(0,0,2,1),
               comDotRef,
               waistAngVelRef.block(0,0,2,1),
               flexAngVelRef.block(0,0,2,1);
    stateRefSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkRef));

    // Equilibrium state reconstruction
    stateObservation::Vector xeq;
    xeq.resize(stateSize_);
    xeq <<     0,
               0,
               comRef(2),
               waistOriRef.block(0,0,2,1),
               flexOriRef.block(0,0,2,1),
               comDotRef,
               waistAngVelRef.block(0,0,2,1),
               flexAngVelRef.block(0,0,2,1);

    // State error
    stateObservation::Vector dxk;
    dxk.resize(stateSize_);
    dxk=xk-xkRef;
    stateErrorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(dxk));

    /// Computing control

    boost::posix_time::ptime tic = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::ptime toc1  = boost::posix_time::microsec_clock::local_time();
    stateObservation::Vector u;
    u.resize(controlSize_);
    u.setZero();
    switch (nbSupport)
    {
        case 0: // No support
        {
            preTask_ <<  -gain*dxk.block(0,0,5,1);
        }
        break;
        case 1: // Single support
        {
             if(nbSupport!=nbSupport_ || computed_==false || fixedGains_!=true) // || comRef!=comRef_)
             {
                // Spring and damping
                Kth_ <<   kts_,0,0,
                          0,kts_,0,
                          0,0,kts_;
                Kdth_ <<  ktd_,0,0,
                          0,ktd_,0,
                          0,0,ktd_;

                // Computing model
                computeDynamicsMatrix(xeq.block(0,0,3,1),Kth_,Kdth_,time);
                controller_.setDynamicsMatrices(A_,B_);
                nbSupport_=nbSupport;
                computed_=true;
                comRef_=comRef;
             }
             // Computing control
             controller_.setState(dxk,time);
             u=controller_.getControl(time);
             u+=perturbationAcc;
             preTask_+=dt_*u;
             xSimu_=xk;
        }
        break;
        case 2 : // Double support
        {
             // fixedGains_=false;
              if(nbSupport!=nbSupport_ || computed_ == false || fixedGains_!=true) // || comRef!=comRef_)
              {

                  stateObservation::Vector3 tc; tc.setZero();
                  tc= convertVector<stateObservation::Vector>(supportPos1_).block(0,0,3,1)-convertVector<stateObservation::Vector>(supportPos2_).block(0,0,3,1);

                  stateObservation::Matrix3 Kts; Kts.setZero();
                  stateObservation::Matrix3 Kfs; Kfs.setZero();
                  stateObservation::Matrix3 Ktd; Ktd.setZero();
                  stateObservation::Matrix3 Kfd; Kfd.setZero();
                  Kts<<    kts_,0,0,
                            0,kts_,0,
                            0,0,kts_;
                  Kfs <<    kfs_,0,0,
                            0,kfs_,0,
                            0,0,kfs_;
                  Kth_=2*Kts-0.5*Kfs*kine::skewSymmetric2(tc);
                  Ktd<<      ktd_,0,0,
                             0,ktd_,0,
                             0,0,ktd_;
                  Kfd <<    kfd_,0,0,
                            0,kfd_,0,
                            0,0,kfd_;
                  Kdth_=2*Ktd-0.5*Kfd*kine::skewSymmetric2(tc);

                  // TODO: when feet are not aligned along the y axis

                  computeDynamicsMatrix(xeq.block(0,0,3,1),Kth_,Kdth_,time);
                  controller_.setDynamicsMatrices(A_,B_);
                  nbSupport_=nbSupport;
                  computed_=true;
                  comRef_=comRef;
              }

              controller_.setState(dxk,time);
              u=controller_.getControl(time);
              u+=perturbationAcc;
              preTask_+=dt_*u;
        }
        break;
        default: throw std::invalid_argument("Only 0, 1 and 2 number of supports cases are developped");
    };
    boost::posix_time::ptime toc  = boost::posix_time::microsec_clock::local_time();

    /// Post treatments


   boost::posix_time::time_duration diff = toc - tic;
   diff.total_microseconds();
   boost::posix_time::time_duration diff1 = toc1 - tic;
   diff1.total_microseconds();
   //std::cout << "Computation time:" << diff.total_microseconds() << std::endl << diff1.total_microseconds() << std::endl;
   stateObservation::Vector computationTime; computationTime.resize(2);
   computationTime << diff.total_microseconds(),
                      diff1.total_microseconds();
   computationTimeSOUT.setConstant(convertVector<dynamicgraph::Vector>(computationTime));

    // Extended state reconstruction
    stateObservation::Vector extxk;
    extxk.resize(18);
    extxk <<    com,
                waistOriVect,
                flexOriVect,
                comDot,
                waistAngVel,
                flexAngVelVect;
    stateExtendedSOUT_.setConstant (convertVector<dynamicgraph::Vector>(extxk));
    stateExtendedSOUT_.setTime (time);

    // State in the world frame
    stateObservation::Vector xkworld;
    xkworld.resize(stateSize_);
    xkworld <<  flexOri*com+tflex,
                (kine::rotationMatrixToRotationVector(flexOri*waistOri)).block(0,0,2,1),
                (flexOriVect).block(0,0,2,1),
                kine::skewSymmetric(flexAngVelVect)*flexOri*com+flexOri*comDot+dtflex,
                (flexAngVelVect+flexOri*waistAngVel).block(0,0,2,1),
                (flexAngVelVect).block(0,0,2,1);
    stateWorldSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkworld));

    // Energy
    double Etot, Eflex, Ecom, Ewaist;
    Eflex=0.5*Kth_(0,0)*flexOriVect.squaredNorm();
    Ecom=0.5*constm_*(xkworld.block(7,0,3,1)).squaredNorm();
    Ewaist=0.5*angularmomentum.dot(waistAngVel);
    Etot=Eflex+Ecom+Ewaist;
    stateObservation::Vector energy;
    energy.resize(4);
    energy <<   Etot,
                Ecom,
                Ewaist,
                Eflex;
    energySOUT_.setConstant(convertVector<dynamicgraph::Vector>(energy));

    stateObservation::Vector error;
    error.resize(controlSize_);
    error.block(0,0,5,1)=dxk.block(0,0,5,1);
    errorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(error));
    errorSOUT_.setTime (time);

    controlSOUT_.setConstant (convertVector<dynamicgraph::Vector>(u));
    controlSOUT_.setTime (time);
    gainSOUT.setConstant (convertMatrix<dynamicgraph::Matrix>(controller_.getLastGain()));
    AmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(A_));
    BmatrixSOUT.setConstant(convertMatrix<dynamicgraph::Matrix>(B_));

    // Validation of the model
    xpredicted_=A_*dxk+B_*u+xk;
    statePredictionSOUT_.setConstant(convertVector<dynamicgraph::Vector>(xpredicted_));

    //stateObservation::Vector xSimu=xSimu_;
    stateObservation::Vector a, b, c;
    c.resize(14); c.setZero();
    a.resize(stateSize_); a.setZero();
    a=xSimu_-c;
    b.resize(stateSize_); b.setZero();
    b=A_*a+B_*u+c;
    xSimu_=b;
    stateSimulationSOUT_.setConstant(convertVector<dynamicgraph::Vector>(xSimu_));

    stateObservation::Vector modelError;
    modelError.resize(stateSize_);
    modelError.setZero();
    modelError=xpredicted_-dxk;
    stateModelErrorSOUT_.setConstant(convertVector<dynamicgraph::Vector>(modelError));

    /// Computing task
    task.resize (taskSize_);
    int i;
    for (i=0;i<controlSize_;i++)
    {
        task [i].setSingleBound (preTask_(i)+perturbationVel(i));
    }
    for (i=controlSize_;i<taskSize_;i++)
    {
        task [i].setSingleBound (preTask_(i-taskSize_+controlSize_)+perturbationVel(i-taskSize_+controlSize_));
    }

    return task;
  }

  Matrix& HRP2LQRTwoDofCoupledStabilizer::computeJacobian(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;

    const stateObservation::Matrix & jacobianCom=convertMatrix<stateObservation::Matrix>(jacobianComSIN_(time));
    const stateObservation::Matrix & jacobianWaist=convertMatrix<stateObservation::Matrix>(jacobianWaistSIN_(time));
    const stateObservation::Matrix & jacobianChest=convertMatrix<stateObservation::Matrix>(jacobianChestSIN_(time));

    stateObservation::Matrix jacobianWaistOri = jacobianWaist.block(3,0,2,jacobianWaist.cols());
    stateObservation::Matrix jacobianChestOri = jacobianChest.block(3,0,2,jacobianChest.cols());

    stateObservation::Matrix preJacobian;
    preJacobian.resize(jacobianCom.rows()+jacobianWaistOri.rows()+jacobianChestOri.rows(),jacobianCom.cols());

    preJacobian.block(0,0,jacobianCom.rows(),jacobianCom.cols())= jacobianCom;
    preJacobian.block(jacobianCom.rows(),0,jacobianWaistOri.rows(),jacobianWaistOri.cols())= jacobianWaistOri;
    preJacobian.block(jacobianCom.rows()+jacobianWaistOri.rows(),0,jacobianChestOri.rows(),jacobianChestOri.cols())= jacobianChestOri;

    jacobian = convertMatrix<dynamicgraph::Matrix>(preJacobian);

    return jacobian;
  }


  void HRP2LQRTwoDofCoupledStabilizer::computeDynamicsMatrix(const stateObservation::Vector3 cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time)
  {
    double g = stateObservation::cst::gravityConstant;
    double m = hrp2Mass_;

    /// State in the local frame

    stateObservation::Matrix I;
    if(constantInertia_!=true)
    {
        I = computeInert(cl,time);
    }
    else
    {
        I=I_;
    }
    stateObservation::Matrix3 identity;
    identity.setIdentity();

    stateObservation::Matrix3 ddomega_cl, ddomega_omegach, ddomega_omega, ddomega_dcl,
                              ddomega_domegach, ddomega_domega, ddomega_ddcl, ddomega_ddomegach;

    // usefull variables for code factorisation
    stateObservation::Vector3 uz;
    uz <<     0,
              0,
              1;

    stateObservation::Matrix Inertia;
    Inertia = I;
    Inertia -= m * kine::skewSymmetric2(cl);
    Inertia = Inertia.inverse();

    stateObservation::Vector3 v;
    stateObservation::Matrix3 dv;
    v=g*m*kine::skewSymmetric(uz)*cl;
    dv=g*m*kine::skewSymmetric(uz);
    ddomega_cl=Inertia*(dv-2*m*kine::skewSymmetric(cl)*kine::skewSymmetric(Inertia*v)+m*kine::skewSymmetric(Inertia*v)*kine::skewSymmetric(cl));
    ddomega_omegach=g*m*I_*kine::skewSymmetric(kine::skewSymmetric(cl)*uz)-g*m*kine::skewSymmetric(I_*kine::skewSymmetric(cl)*uz);
    ddomega_omega=-Inertia*(Kth+g*m*kine::skewSymmetric(uz)*kine::skewSymmetric(cl));
    ddomega_dcl.setZero();
    ddomega_domegach.setZero();
    ddomega_domega=-Inertia*Kdth;

    ddomega_ddcl=-m*Inertia*kine::skewSymmetric(cl);
    ddomega_ddomegach=-Inertia*I;

    // A_ and B_ computation
    A_.setZero();
    A_.block(0,7,3,3)=identity;
    A_.block(3,10,2,2)=identity.block(0,0,2,2);
    A_.block(5,12,2,2)=identity.block(0,0,2,2);
    A_.block(12,0,2,3)=ddomega_cl.block(0,0,2,3);
    A_.block(12,3,2,2)=ddomega_omegach.block(0,0,2,2);
    A_.block(12,5,2,2)=ddomega_omega.block(0,0,2,2);
    A_.block(12,7,2,3)=ddomega_dcl.block(0,0,2,3);
    A_.block(12,10,2,2)=ddomega_domegach.block(0,0,2,2);
    A_.block(12,12,2,2)=ddomega_domega.block(0,0,2,2);

    stateObservation::Matrix Identity(stateObservation::Matrix::Zero(stateSize_,stateSize_));
    Identity.setIdentity();

    A_.noalias() = dt_ * A_ + Identity;

    B_.setZero();
    B_.block(7,0,3,3)=identity;
    B_.block(10,3,2,2)=identity.block(0,0,2,2);
    B_.block(12,0,2,3)=ddomega_ddcl.block(0,0,2,3);
    B_.block(12,3,2,2)=ddomega_ddomegach.block(0,0,2,2);

    B_.noalias() = dt_* B_;

  }


  stateObservation::Matrix3 HRP2LQRTwoDofCoupledStabilizer::computeInert(const stateObservation::Vector& cl, const int& inTime)
{
    const stateObservation::Matrix& inertiaSotWaistFrame=convertMatrix<stateObservation::Matrix>(inertiaSIN.access(inTime));
    const MatrixHomogeneous& waistHomo = waistHomoSIN_.access(inTime);
    Vector waistPos(3);
    waistHomo.extract(waistPos);

    double m=hrp2Mass_;
    stateObservation::Vector3 wl=convertVector<stateObservation::Vector>(waistPos);


    stateObservation::Matrix3 inertiaControlFrame, inertiaComFrame, inertiaWaistFrame;

    inertiaWaistFrame = inertiaSotWaistFrame.block(3,3,3,3);
    inertiaComFrame = inertiaWaistFrame + m * kine::skewSymmetric2(cl-wl);

    inertiaSOUT.setConstant (convertMatrix<dynamicgraph::Matrix>(inertiaComFrame));
    I_=inertiaComFrame;
    return inertiaComFrame;
}

} // namespace sotStabilizer
