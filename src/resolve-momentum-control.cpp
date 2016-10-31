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

#include <state-observation/flexibility-estimation/imu-elastic-local-frame-dynamical-system.hpp>

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
    zmpRefSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::zmpRef"),
    momentaSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::momenta"),
    inertiaSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::inertia"),
    dinertiaSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(matrix)::dinertia"),
    estimatorStateSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::estimatorState"),
    estimatorInputSIN_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::input(vector)::estimatorInput"),
    on_(true), dt_(0.005)
  {

    // Signals registration
    signalRegistration (zmpRefSIN_);
    signalRegistration (momentaSIN_);
    signalRegistration (inertiaSIN_);
    signalRegistration (dinertiaSIN_);
    signalRegistration (estimatorStateSIN_);
    signalRegistration (estimatorInputSIN_);

    // Set dependencies
        // taskSOUT dependencies
    //taskSOUT.addDependency (*); for all input signals
    taskSOUT.addDependency (zmpRefSIN_);
    taskSOUT.addDependency (momentaSIN_);
    taskSOUT.addDependency (inertiaSIN_);
    taskSOUT.addDependency (dinertiaSIN_);
    taskSOUT.addDependency (estimatorStateSIN_);
    taskSOUT.addDependency (estimatorInputSIN_);

        // jacobianSOUT dependencies
    //jacobianSOUT.addDependency (*); for all jacobian input signals

    // Setting methods for output signals
    taskSOUT.setFunction (boost::bind(&ResolveMomentumControl::computeTask,this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&ResolveMomentumControl::computeJacobian,this,_1,_2));

  }

 /// Compute the task
  VectorMultiBound&  ResolveMomentumControl::computeTask(VectorMultiBound& task, const int& time)
  {
    const stateObservation::Vector & zmpRef = convertVector<stateObservation::Vector>(zmpRefSIN_.access(time));
    const stateObservation::Vector & momenta = convertVector<stateObservation::Vector>(momentaSIN_.access(time));
    const stateObservation::Matrix & inertia = convertMatrix<stateObservation::Matrix>(inertiaSIN_.access(time));
    const stateObservation::Matrix & dinertia = convertMatrix<stateObservation::Matrix>(dinertiaSIN_.access(time));
    const stateObservation::Vector & estimatorState = convertVector<stateObservation::Vector>(estimatorStateSIN_.access(time));
    const stateObservation::Vector & estimatorInput = convertVector<stateObservation::Vector>(estimatorInputSIN_.access(time));

    stateObservation::Vector3 gmuz, oriV, angVel, angAcc, Fc;
    stateObservation::Vector3 cl, dcl, ddcl;
    stateObservation::Matrix3 R;

    gmuz << 0,
            0,
            9.81*hrp2::m;

    // Estimator state
    oriV = estimatorState.segment<3>(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::ori);
    R = kine::rotationVectorToRotationMatrix(oriV);
    angVel = estimatorState.segment<3>(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::angVel);
    angAcc.setZero();

    // Estimator input
    cl = estimatorInput.segment<3>(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input::posCom);
    dcl = estimatorInput.segment<3>(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input::velCom);
    ddcl = estimatorInput.segment<3>(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input::accCom);
    Fc.setZero();
    for (int i=0; i<hrp2::contact::nbModeledMax; ++i)
        Fc += estimatorState.segment(stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state::fc+6*i,6).block<3,1>(0,0);

    // Reference of angular momentum in the world frame
    stateObservation::Vector3 sigmaRef;
    sigmaRef = kine::skewSymmetric(zmpRef)*Fc + kine::skewSymmetric(R*cl)*gmuz;
    sigmaRef[2] = 0;

    // Reference of angular momentum in the control frame
    stateObservation::Vector3 sigmalRef;
    sigmalRef = R.transpose()*(sigmaRef-(kine::skewSymmetric(angVel)*R*momenta+kine::skewSymmetric(angVel)*R*inertia*R.transpose()*angVel+R*dinertia*R.transpose()*angVel+R*inertia*R.transpose()*angAcc));

    stateObservation::Vector3 preTask;
    preTask = sigmalRef - momenta;
    for (unsigned i=0;i<3;i++)
        task [i].setSingleBound (preTask(i));
    return task;
  }

  Matrix& ResolveMomentumControl::computeJacobian(Matrix& jacobian, const int& time)
  {
    return jacobian;
  }

} // namespace sotStabilizer
