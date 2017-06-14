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
#include "boost/date_time/posix_time/posix_time.hpp"

#include <iostream>

//#define DEBUG

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

  double HRP2SMCStabilizer::constm_ = 56.8;

  const unsigned stateSize_=9;
  const unsigned controlSize_=6;
  const unsigned taskSize_=9;

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
    waistHomoSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::waistHomo"),
    flexOriVectSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::flexOriVect"),
    comDotSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comDot"),
    waistAngVelSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::waistAngVel"),
    flexAngVelVectSIN_(NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::flexAngVelVect"),
    comRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comRef"),
    waistOriRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::waistOriRef"),
    flexOriRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::flexOriRef"),
    comDotRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comDotRef"),
    waistVelRefSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::waistVelRef"),
    flexAngVelRefSIN_(NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::flexAngVelRef"),
    jacobianComSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::Jcom"),
    jacobianWaistSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::Jwaist"),
    jacobianChestSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::Jchest"),
    perturbationVelSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::perturbationVel"),
    perturbationAccSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::perturbationAcc"),
    comBiasSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::comBias"),
    controlGainSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(double)::controlGain"),
    supportPos1SIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::supportPos1"),
    supportPos2SIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(vector)::supportPos2"),
    inertiaSIN_ (NULL, "HRP2SMCStabilizer("+inName+")::input(matrix)::intertia"),
    stateSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::state"),
    integratedTaskSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::integratedTask"),
    stateErrorSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::stateError"),
    stateRefSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::stateRef"),
    errorSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::error"),
    controlSOUT_ ("HRP2SMCStabilizer("+inName+")::output(vector)::control"),
    nbSupportSIN_ (0x0,"HRP2SMCStabilizer("+inName+")::input(unsigned)::nbSupport"),
    dt_ (.005), on_ (false), computed_(false), fixedGains_(true),
    A_(stateSize_,stateSize_), B_(stateSize_,controlSize_),
    SMCController_(stateSize_,controlSize_),
    LQRController_(stateSize_,controlSize_)
  {

    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (nbSupportSIN_);
    signalRegistration (waistHomoSIN_);
    signalRegistration (flexOriVectSIN_);
    signalRegistration (comDotSIN_);
    signalRegistration (waistAngVelSIN_);
    signalRegistration (flexAngVelVectSIN_);
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
    signalRegistration (supportPos1SIN_);
    signalRegistration (supportPos2SIN_);
    signalRegistration (inertiaSIN_);
    signalRegistration (stateSOUT_);
    signalRegistration (integratedTaskSOUT_);
    signalRegistration (stateRefSOUT_);
    signalRegistration (stateErrorSOUT_);
    signalRegistration (errorSOUT_);
    signalRegistration (controlSOUT_);

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
    taskSOUT.addDependency (supportPos1SIN_);
    taskSOUT.addDependency (supportPos2SIN_);
    taskSOUT.addDependency (inertiaSIN_);
        // jacobianSOUT dependencies
    jacobianSOUT.addDependency (jacobianComSIN_);
    jacobianSOUT.addDependency (jacobianWaistSIN_);

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


    nbSupportSIN_.setConstant (2);
    nbSupportSIN_.setTime (0);
    nbSupport_=2;

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

    Vector vect;
    stateObservation::Vector vec;
    vec.resize(2); vec.setZero();
    vect.resize(3); vect.setZero();
    comDotSIN_.setConstant(vect);
    flexOriVectSIN_.setConstant(vect);
    flexAngVelVectSIN_.setConstant(vect);
    vect.resize(6); vect.setZero();
    waistAngVelSIN_.setConstant(vect);
    waistAngVelSIN_.setTime(0);

    preTask_.resize(controlSize_);
    preTask_.setZero();
    integratedTask_.resize(controlSize_);
    integratedTask_.setZero();

    hrp2Mass_ = constm_;

    dynamicgraph::Vector zero(6); zero.setZero();
    controlSOUT_.setConstant(zero);

    vect.resize(5);
    vect.setZero();
    perturbationVelSIN_.setConstant(vect);
    perturbationAccSIN_.setConstant(vect);

    SMCController_.setStateDerivativeRef(stateObservation::Vector::Zero(stateSize_)); // stabilization around an equilibrium
    stateObservation::Matrix lambdaa, alpha0, lambda0;
    stateObservation::Vector alpha;
    lambdaa.resize(controlSize_,controlSize_);
    alpha0.resize(controlSize_,SMCController_.getUnderActuatedSize());
    lambda0.resize(controlSize_,SMCController_.getUnderActuatedSize());
    alpha.resize(controlSize_);
    lambdaa << 1,0,0,0,0,0,
               0,1,0,0,0,0,
               0,0,0.1,0,0,0,
               0,0,0,1,0,0,
               0,0,0,0,1,0,
               0,0,0,0,0,0.1;
//               0,0,0,10,0,0,
//               0,0,0,0,10,0,
//               0,0,0,0,0,10;
    lambda0 << 0,1,0,
               -0,0,0,
               0,0,0,
               0,0,0,
               0,0,0,
               0,0,0;
//               0,1,0,
//               -1,0,0,
//               0,0,1;
    alpha0 <<  0,1,0,
               0,0,0,
               0,0,0,
               0,0,0,
               0,0,0,
               0,0,0;
//               0,1,0,
//               -1,0,0,
//               0,0,1;
    alpha  << 1,
              1,
              0,
              1,
              1,
              0;
    SMCController_.setLambdaa(lambdaa);
    SMCController_.setLambda0(lambda0);
    SMCController_.setAlpha0(alpha0);
    SMCController_.setAlpha(alpha);

    // For the LQR controller
    kts_=60;
    ktd_=6.5;
    kfs_=600;
    kfd_=65;
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

#ifdef DEBUG
      std::cout<<"Getting state" << std::endl;
#endif // DEBUG*

    // State
    const stateObservation::Vector & comBias = convertVector<stateObservation::Vector>(comBiasSIN_.access(time));
    const stateObservation::Vector & com = convertVector<stateObservation::Vector>(comSIN_.access(time));
    const Matrix4& waistHomo = convertMatrix<stateObservation::Matrix>(waistHomoSIN_ (time));
    const stateObservation::Vector & flexOriVect = convertVector<stateObservation::Vector>(flexOriVectSIN_.access(time));
    const stateObservation::Vector & comDot = convertVector<stateObservation::Vector>(comDotSIN_ (time));
    const stateObservation::Vector waistAngVelIn=convertVector<stateObservation::Vector>(waistAngVelSIN_ (time));
    const stateObservation::Vector & waistAngVel = waistAngVelIn.block(3,0,3,1);
    const stateObservation::Vector & flexAngVelVect = convertVector<stateObservation::Vector>(flexAngVelVectSIN_.access(time));

#ifdef DEBUG
      std::cout<<"Getting state reference" << std::endl;
#endif // DEBUG*

    // State Reference
    const stateObservation::Vector & comRef = convertVector<stateObservation::Vector>(comRefSIN_ (time));
    const stateObservation::Vector & waistOriRef = convertVector<stateObservation::Vector>(waistOriRefSIN_.access(time));
    const stateObservation::Vector & flexOriRef = convertVector<stateObservation::Vector>(flexOriRefSIN_.access(time));
    const stateObservation::Vector & comDotRef = convertVector<stateObservation::Vector>(comDotRefSIN_ (time));
    const stateObservation::Vector & waistAngVelRef = convertVector<stateObservation::Vector>(waistVelRefSIN_ (time));
    const stateObservation::Vector & flexAngVelRef = convertVector<stateObservation::Vector>(flexAngVelRefSIN_.access(time));

    std::cout << "flexOriRef=" << flexOriRef.transpose() << std::endl;

    // Others
    const double& gain = controlGainSIN_.access (time);

    // Determination of the number of support
    unsigned int nbSupport=nbSupportSIN_.access(time);
    if (!on_) nbSupport=0;
    supportPos1_=supportPos1SIN_.access(time);
    supportPos2_=supportPos2SIN_.access(time);

    // Waist orientation
    Matrix3 waistOri=waistHomo.block(0,0,3,3);
    stateObservation::Vector3 waistOriVect;
    waistOriVect=kine::rotationMatrixToRotationVector(waistOri);

    // flex orientation
    Matrix3 flexOri=kine::rotationVectorToRotationMatrix(flexOriVect);

#ifdef DEBUG
      std::cout<<"State reconstruction" << std::endl;
#endif // DEBUG*

    // State reconstruction
    stateObservation::Vector xk;
    xk.resize(2*stateSize_);
    xk <<   com,
            waistOriVect,
            flexOriVect,
            comDot,
            waistAngVel,
            flexAngVelVect;
    stateSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xk));

#ifdef DEBUG
      std::cout<<"State reference reconstruction" << std::endl;
#endif // DEBUG*

    // Reference reconstruction
    stateObservation::Vector xkRef;
    xkRef.resize(2*stateSize_);
    xkRef <<   comRef,
               waistOriRef,
               flexOriRef,
               comDotRef,
               waistAngVelRef,
               flexAngVelRef;
    stateRefSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xkRef));

#ifdef DEBUG
      std::cout<<"State error reconstruction" << std::endl;
#endif // DEBUG*

    // State error
    stateObservation::Vector dxk;
    dxk.resize(2*stateSize_);
    dxk=xk-xkRef;
    stateErrorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(dxk));

#ifdef DEBUG
      std::cout<<"Equilibrium state reconstruction" << std::endl;
#endif // DEBUG*

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

    /// Computing control

#ifdef DEBUG
      std::cout<<"Compute control" << std::endl;
#endif // DEBUG*

    stateObservation::Vector u;
    u.resize(controlSize_);
    u.setZero();
    switch (nbSupport)
    {
        case 0: // No support
        {
#ifdef DEBUG
      std::cout<< "O support case" << std::endl;
#endif // DEBUG*

            preTask_ <<  -gain*dxk.block(0,0,controlSize_,1);
        }
        break;
        case 1: // Single support
    {
#ifdef DEBUG
      std::cout<< "1 support case" << std::endl;
#endif // DEBUG*

            // Compute the LQR gains
            if(nbSupport!=nbSupport_ || computed_==false || fixedGains_!=true)
            {
                // Spring and damping
                Kth_ << kts_,0,0,
                        0,kts_,0,
                        0,0,kts_;
                Kdth_ << ktd_,0,0,
                         0,ktd_,0,
                         0,0,ktd_;
                // Computing model
                computeDynamicsMatrix(xeq.block(0,0,3,1),Kth_,Kdth_,time);
                LQRController_.setDynamicsMatrices(A_,B_);
                nbSupport_=nbSupport;
                computed_=true;
            }

            // Compute next control
            SMCController_.setState(xk.segment(0,stateSize_),time);
            SMCController_.setStateDerivative(xk.segment(stateSize_,stateSize_));
            SMCController_.setStateRef(xkRef);
            u=SMCController_.getControl(time);
            preTask_+=dt_*u;
        }
        break;
        case 2 : // Double support
        {
#ifdef DEBUG
      std::cout<< "2 support case" << std::endl;
#endif // DEBUG*

            if(nbSupport!=nbSupport_ || computed_ == false || fixedGains_!=true)
            {

                stateObservation::Vector3 tc; tc.setZero();
                tc= convertVector<stateObservation::Vector>(supportPos1_).block(0,0,3,1)-convertVector<stateObservation::Vector>(supportPos2_).block(0,0,3,1);

                stateObservation::Matrix3 Kts; Kts.setZero();
                stateObservation::Matrix3 Kfs; Kfs.setZero();
                stateObservation::Matrix3 Ktd; Ktd.setZero();
                stateObservation::Matrix3 Kfd; Kfd.setZero();
                Kts<<  kts_,0,0,
                       0,kts_,0,
                       0,0,kts_;
                Kfs << kfs_,0,0,
                       0,kfs_,0,
                       0,0,kfs_;
                Kth_=2*Kts-0.5*Kfs*kine::skewSymmetric2(tc);
                Ktd<<   ktd_,0,0,
                        0,ktd_,0,
                        0,0,ktd_;
                Kfd << kfd_,0,0,
                       0,kfd_,0,
                       0,0,kfd_;
                Kdth_=2*Ktd-0.5*Kfd*kine::skewSymmetric2(tc);

                // TODO: when feet are not aligned along the y axis

                computeDynamicsMatrix(xeq.block(0,0,3,1),Kth_,Kdth_,time);
                LQRController_.setDynamicsMatrices(A_,B_);
                nbSupport_=nbSupport;
                computed_=true;
            }
            // Compute next control
            SMCController_.setState(xk.segment(0,stateSize_),time);
            SMCController_.setStateDerivative(xk.segment(stateSize_,stateSize_));
            SMCController_.setStateRef(xkRef);
            u=SMCController_.getControl(time);
            preTask_+=dt_*u;
        }
        break;
        default: throw std::invalid_argument("Only 0, 1 and 2 number of supports cases are developped");
    };

    /// Post treatments

#ifdef DEBUG
      std::cout<<"Setting output signals" << std::endl;
#endif // DEBUG*

    stateObservation::Vector error;
    error.resize(controlSize_);
    error.block(0,0,5,1)=dxk.block(0,0,5,1);
    errorSOUT_.setConstant (convertVector<dynamicgraph::Vector>(error));
    errorSOUT_.setTime (time);

    controlSOUT_.setConstant (convertVector<dynamicgraph::Vector>(u));
    controlSOUT_.setTime (time);

    integratedTask_+=dt_*preTask_;
    integratedTaskSOUT_.setConstant (convertVector<dynamicgraph::Vector>(integratedTask_));
    integratedTaskSOUT_.setTime (time);


#ifdef DEBUG
      std::cout<<"Setting task" << std::endl;
#endif // DEBUG*

    /// Computing task
    task.resize (taskSize_);
    int i;
    for (i=0;i<controlSize_;i++)
    {
        task [i].setSingleBound (preTask_(i));//+perturbationVel(i));
    }
    for (i=controlSize_;i<taskSize_;i++)
    {
        task [i].setSingleBound (preTask_(i-taskSize_+controlSize_));//+perturbationVel(i-taskSize_+controlSize_));
    }

#ifdef DEBUG
      std::cout<<"The task is set" << std::endl;
#endif // DEBUG*

    return task;
  }

  Matrix& HRP2SMCStabilizer::computeJacobian(Matrix& jacobian, const int& time)
  {
    typedef unsigned int size_t;

#ifdef DEBUG
      std::cout<<"Getting input jacobians" << std::endl;
#endif // DEBUG*

    const stateObservation::Matrix & jacobianCom=convertMatrix<stateObservation::Matrix>(jacobianComSIN_(time));
    const stateObservation::Matrix & jacobianWaist=convertMatrix<stateObservation::Matrix>(jacobianWaistSIN_(time));
    const stateObservation::Matrix & jacobianChest=convertMatrix<stateObservation::Matrix>(jacobianChestSIN_(time));

#ifdef DEBUG
      std::cout<<"Beggining to construct the task jacobian" << std::endl;
#endif // DEBUG*

    stateObservation::Matrix jacobianWaistOri = jacobianWaist.block(3,0,2,jacobianWaist.cols());
    stateObservation::Matrix jacobianChestOri = jacobianChest.block(3,0,2,jacobianChest.cols());

    stateObservation::Matrix preJacobian;
    preJacobian.resize(jacobianCom.rows()+jacobianWaistOri.rows()+jacobianChestOri.rows(),jacobianCom.cols());

    preJacobian.block(0,0,jacobianCom.rows(),jacobianCom.cols())= jacobianCom;
    preJacobian.block(jacobianCom.rows(),0,jacobianWaistOri.rows(),jacobianWaistOri.cols())= jacobianWaistOri;
    preJacobian.block(jacobianCom.rows()+jacobianWaistOri.rows(),0,jacobianChestOri.rows(),jacobianChestOri.cols())= jacobianChestOri;

    jacobian = convertMatrix<dynamicgraph::Matrix>(preJacobian);

#ifdef DEBUG
      std::cout<<"The jacobian is set" << std::endl;
#endif // DEBUG*

    return jacobian;
  }

    void HRP2SMCStabilizer::computeDynamicsMatrix(const stateObservation::Vector3 cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time)
    {
        double g = stateObservation::cst::gravityConstant;
        double m = hrp2Mass_;

        /// State in the local frame

        I_ = computeInert(cl,time);

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
        Inertia = I_;
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
        ddomega_ddomegach=-Inertia*I_;

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


    stateObservation::Matrix3 HRP2SMCStabilizer::computeInert(const stateObservation::Vector& cl, const int& inTime)
    {
        const stateObservation::Matrix& inertiaSotWaistFrame=convertMatrix<stateObservation::Matrix>(inertiaSIN_.access(inTime));
        const MatrixHomogeneous& waistHomo = waistHomoSIN_.access(inTime);
        Vector waistPos(3);
        waistHomo.extract(waistPos);

        double m=hrp2Mass_;
        stateObservation::Vector3 wl=convertVector<stateObservation::Vector>(waistPos);

        stateObservation::Matrix3 inertiaComFrame, inertiaWaistFrame;

        inertiaWaistFrame = inertiaSotWaistFrame.block(3,3,3,3);
        inertiaComFrame = inertiaWaistFrame + m * kine::skewSymmetric2(cl-wl);

        return inertiaComFrame;
    }


} // namespace sotStabilizer
