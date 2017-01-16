/*
 *  Copyright 2014 CNRS
 *
 *  Alexis Mifsud
 */

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <sot-stabilizer/prototyping/non-linear-rotational-table-cart-device.hh>
#include <dynamic-graph/command-bind.h>
#include "command-increment_NonLinearRotationalTableCartDevice.hh"
#include <sot/core/matrix-homogeneous.hh>

#include <iostream>

using dynamicgraph::command::makeDirectSetter;
using dynamicgraph::command::makeDirectGetter;

namespace sotStabilizer
{
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(NonLinearRotationalTableCartDevice, "NonLinearRotationalTableCartDevice");

NonLinearRotationalTableCartDevice::NonLinearRotationalTableCartDevice(const std::string& inName) :
  Entity(inName),
  controlSIN_(NULL, "NonLinearRotationalTableCartDevice("+inName+")::input(vector)::control"),
  stateSOUT_("NonLinearRotationalTableCartDevice("+inName+")::output(vector)::state"),
  controlStateSOUT_("NonLinearRotationalTableCartDevice("+inName+")::output(vector)::controlState"),
  comSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::com"),
  waistHomoSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::waistHomo"),
  flexOriVectSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexOriVect"),
  comDotSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::comDot"),
  waistAngVelSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::waistAngVel"),
  flexAngVelVectSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexAngVelVect"),
  flexAngAccVectSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexAngAccVect"),
  flexLinAccSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::flexLinAcc"),
  contact1PosSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(MatrixHomogeneous)::contact1Pos"),
  contact1ForcesSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::contact1Forces"),
  contact2PosSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(MatrixHomogeneous)::contact2Pos"),
  contact2ForcesSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::contact2Forces"),
  sumMomentsSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::sumMoments"),
  inertiaSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::inertia"),
  dotInertiaSOUT_ (NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(matrix)::dotInertia"),
  angularMomentumSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::angularMomentum"),
  dotAngularMomentumSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::dotAngularMomentum"),
  stateWorldSOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::stateWorld"),
  energySOUT_(NULL, "HRP2LQRTwoDofCoupledStabilizer("+inName+")::output(vector)::energy"),
  robotMass_(58.0), robotMassInv_(1/58.0), I_ (3,3), cl_(3), contactsNumber_(2), xn_(24), AngMomentum_(3)
{

  Kfe_.resize(3,3);
  Kfv_.resize(3,3);
  Kte_.resize(3,3);
  Ktv_.resize(3,3);
  I_.setZero();

  Kfe_ <<   100,0,0,
            0,100,0,
            0,0,100;

  Kfv_ <<     0.1,0,0,
              0,0.1,0,
              0,0,0.1;

  Kte_ <<    100,0,0,
             0,100,0,
             0,0,100;

  Ktv_ <<     0.1,0,0,
              0,0.1,0,
              0,0,0.1;

  // Register signals into the entity.
  signalRegistration (controlSIN_);
  signalRegistration (stateSOUT_);
  signalRegistration (comSOUT_);
  signalRegistration (waistHomoSOUT_);
  signalRegistration (flexOriVectSOUT_);
  signalRegistration (comDotSOUT_);
  signalRegistration (waistAngVelSOUT_);
  signalRegistration (flexAngVelVectSOUT_);
  signalRegistration (flexAngAccVectSOUT_);
  signalRegistration (flexLinAccSOUT_);
  signalRegistration (contact1PosSOUT_);
  signalRegistration (contact1ForcesSOUT_);
  signalRegistration (contact2PosSOUT_);
  signalRegistration (contact2ForcesSOUT_);
  signalRegistration (sumMomentsSOUT_);
  signalRegistration (inertiaSOUT_);
  signalRegistration (dotInertiaSOUT_);
  signalRegistration (angularMomentumSOUT_);
  signalRegistration (dotAngularMomentumSOUT_);
  signalRegistration (energySOUT_);


//  comSOUT_.addDependency (controlSIN_);
//  waistHomoSOUT_.addDependency (controlSIN_);
//  flexOriVectSOUT_.addDependency (controlSIN_);
//  comDotSOUT_.addDependency (controlSIN_);
//  waistVelSOUT_.addDependency (controlSIN_);
//  flexAngVelVectSOUT_.addDependency (controlSIN_);

  // Set signals as constant to size them
  stateObservation::Vector state;
  state.resize(24);
//  state <<  0.00949046,
//            0,
//            0.80771,
  state <<  0.00949,
            0,
            0.80777699999999997,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            -0.0073,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0;
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state));
  stateSOUT_.setTime(0);

  stateObservation::Vector control; control.resize(5);
  control.setZero();
  controlSIN_.setConstant(convertVector<dynamicgraph::Vector>(control));
  controlSIN_.setTime(0);

  stateObservation::Vector controlState; controlState.resize(14);
  controlState.setZero();
  controlStateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(controlState));
  controlStateSOUT_.setTime(0);

  stateObservation::Vector com;
  com.resize(3);
//  com <<  0.00949046,
//          0,
//          0.80771;
  com <<  0.00949, //0.00965,
          0,
          0.80777699999999997;
  comSOUT_.setConstant(convertVector<dynamicgraph::Vector>(com));
  comSOUT_.setTime(0);

  stateObservation::Matrix4 homoWaist;
  homoWaist <<      0.99998573432883131, -0.0053403256847235764, 0.00010981989355530105, -1.651929567364003e-05,
                    0.0053403915800877009, 0.99998555471196726, -0.00060875707006170711, 0.0008733516988761506,
                    -0.00010656734615829933, 0.00060933486696839291, 0.99999980867719196, 0.64869730032049466,
                    0.0, 0.0, 0.0, 1.0;
  waistHomoSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(homoWaist));
  waistHomoSOUT_.setTime(0);

  dynamicgraph::Vector vect;
  stateObservation::Vector vec;
  vec.resize(2); vec.setZero();
  vect.resize(3); vect.setZero();
  comDotSOUT_.setConstant(vect);
  comDotSOUT_.setTime(0);
  flexOriVectSOUT_.setConstant(vect);
  flexOriVectSOUT_.setTime(0);
  flexAngVelVectSOUT_.setConstant(vect);
  flexAngVelVectSOUT_.setTime(0);
  angularMomentumSOUT_.setConstant(vect);
  AngMomentum_.setZero();
  dotAngularMomentumSOUT_.setConstant(vect);
  vect.resize(6); vect.setZero();
  waistAngVelSOUT_.setConstant(vect);
  waistAngVelSOUT_.setTime(0);

  dynamicgraph::Matrix mat;
  mat.resize(3,3); mat.setZero();
  inertiaSOUT_.setConstant(mat);
  dotInertiaSOUT_.setConstant(mat);

  vect.resize(3); vect.setZero();
  flexAngAccVectSOUT_.setConstant(vect);
  flexAngAccVectSOUT_.setTime(0);
  flexLinAccSOUT_.setConstant(vect);
  flexLinAccSOUT_.setTime(0);

  stateObservation::Matrix4 positionHomo;
  positionHomo  <<  0.99999999999999967, 2.5968437190932057e-08, 6.257471633701753e-11, 0.0,
                    -2.5968437190709582e-08, 0.99999999999999967, -2.5764338781213246e-09, -0.095,
                    -6.2574838892307319e-11, 2.5764338764907862e-09, 1.0000000000000002, 0.0,
                     0.0, 0.0, 0.0, 1.0;
  setContactPosition(0, positionHomo);
    supportPos1_=positionHomo;
  contact1PosSOUT_.setConstant(convertMatrix<dynamicgraph::sot::MatrixHomogeneous>(positionHomo));
  positionHomo  <<  0.99999999999999956, 2.5964823352109814e-08, 6.2554585725676566e-11, 0.0,
                    -2.5964823352024697e-08, 0.99999999999999978, -2.5759706733995109e-09, 0.095,
                    -6.2554685064698801e-11, 2.5759706713355034e-09, 1.0000000000000002, 0.0,
                     0.0, 0.0, 0.0, 1.0;
  setContactPosition(1, positionHomo);
  contact2PosSOUT_.setConstant(convertMatrix<dynamicgraph::sot::MatrixHomogeneous>(positionHomo));

  supportPos2_=positionHomo;

  stateObservation::Vector forces;
  forces.resize(6); forces.setZero();
  contact1ForcesSOUT_.setConstant(convertVector<dynamicgraph::Vector>(forces));
  contact2ForcesSOUT_.setConstant(convertVector<dynamicgraph::Vector>(forces));

  forces.resize(3); forces.setZero();
  sumMomentsSOUT_.setConstant(convertVector<dynamicgraph::Vector>(forces));

  vect.resize(4); vect.setZero();
  energySOUT_.setConstant(vect);

  xn_.setZero();

  // Commands
  std::string docstring;

  // Incr
  docstring =
    "\n"
    "    Integrate dynamics for time step provided as input\n"
    "\n"
    "      take one floating point number as input\n"
    "\n";
  addCommand(std::string("incr"),
             new command::Increment1(*this, docstring));

  // setCartMass
  docstring =
    "\n"
    "    Set robot mass\n"
    "\n";
  addCommand(std::string("setRobotMass"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, double>
             (*this, &NonLinearRotationalTableCartDevice::setRobotMass, docstring));

  // getCartMass
  docstring =
    "\n"
    "    Get robot mass\n"
    "\n";
  addCommand(std::string("getRobotMass"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, double>
             (*this, &NonLinearRotationalTableCartDevice::getRobotMass, docstring));


  // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setKfe"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKfe, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getKfe"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKfe, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setKfv"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKfv, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getKfv"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKfv, docstring));


 // setStiffness
  docstring =
    "\n"
    "    Set stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("setKte"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKte, docstring));

  // getStiffness
  docstring =
    "\n"
    "    Get cart stiffness of the flexibility\n"
    "\n";
  addCommand(std::string("getKte"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKte, docstring));
  // setViscosity
  docstring =
    "\n"
    "    Set viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("setKtv"),
             new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::setKtv, docstring));

  // getViscosity
  docstring =
    "\n"
    "    Get cart viscosity of the flexibility\n"
    "\n";
  addCommand(std::string("getKtv"),
             new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
             (*this, &NonLinearRotationalTableCartDevice::getKtv, docstring));


  // setMomentOfInertia
  docstring =
    "\n"
    "    Set moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("setMomentOfInertia"),
               new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &NonLinearRotationalTableCartDevice::setMomentOfInertia, docstring));

  // setMomentOfInertia
  docstring =
    "\n"
    "    Get moment of inertia around y axis\n"
    "\n";

    addCommand(std::string("getMomentOfInertia"),
               new ::dynamicgraph::command::Getter<NonLinearRotationalTableCartDevice, dynamicgraph::Matrix>
               (*this, &NonLinearRotationalTableCartDevice::getMomentOfInertia, docstring));

    // setState
    docstring =
      "\n"
      "    Set state\n"
      "\n";
    addCommand(std::string("setState"),
               new ::dynamicgraph::command::Setter<NonLinearRotationalTableCartDevice, dynamicgraph::Vector>
               (*this, &NonLinearRotationalTableCartDevice::setState, docstring));

}

NonLinearRotationalTableCartDevice::~NonLinearRotationalTableCartDevice()
{
}

void NonLinearRotationalTableCartDevice::setContactPosition
                                    (unsigned i, const Matrix4 & positionHomo)
{
    stateObservation::Vector6 positionVect;
    positionVect=kine::homogeneousMatrixToVector6(positionHomo);
    opti_.contactPosV.setValue(positionVect.block(0,0,3,1),i);
    opti_.contactOriV.setValue(positionVect.block(3,0,3,1),i);
}

Vector6 NonLinearRotationalTableCartDevice::getContactPosition(unsigned i)
{
    stateObservation::Vector6 contactPosVect;
    contactPosVect  <<  opti_.contactPosV[i],
                        opti_.contactOriV[i];
    return contactPosVect;
}



void NonLinearRotationalTableCartDevice::computeAccelerations
   (const Vector3& positionCom, const Vector3& velocityCom,
    const Vector3& accelerationCom, const Vector3& AngMomentum,
    const Vector3& dotAngMomentum,
    const Matrix3& Inertia, const Matrix3& dotInertia,
    const IndexedMatrixArray& contactPosV,
    const IndexedMatrixArray& contactOriV,
    const Vector3& position, const Vector3& linVelocity, Vector3& linearAcceleration,
    const Vector3 &oriVector ,const Matrix3& orientation,
    const Vector3& angularVel, Vector3& angularAcceleration)
{
    opti_.skewV=kine::skewSymmetric(angularVel);
    opti_.skewV2=kine::skewSymmetric2(angularVel);
    opti_.skewVR=opti_.skewV * orientation;
    opti_.skewV2R=opti_.skewV2 * orientation;

    opti_.rFlexT=orientation.transpose();

    computeElastContactForcesAndMoments (contactPosV, contactOriV,
                      position, linVelocity, oriVector, orientation,
                         angularVel, opti_.fc, opti_.tc);

    opti_.wx2Rc=opti_.skewV2R*positionCom;
    opti_._2wxRv=2*opti_.skewVR*velocityCom;
    opti_.Ra = orientation*accelerationCom;
    opti_.Rc = orientation * positionCom;
    opti_.Rcp = opti_.Rc+position;
    opti_.RIRT = orientation*Inertia*opti_.rFlexT;

    opti_.vf =robotMassInv_*opti_.fc;
    opti_.vf.noalias() -= opti_.Ra;
    opti_.vf.noalias() -= opti_._2wxRv;
    opti_.vf.noalias() -= opti_.wx2Rc;
    opti_.vf.noalias() -= cst::gravity;

    opti_.vt =opti_.tc;
    opti_.vt.noalias() -= opti_.skewV * (opti_.RIRT * angularVel);
    opti_.vt.noalias() -= orientation * (dotInertia * (opti_.rFlexT * angularVel)+dotAngMomentum) ;
    opti_.vt.noalias() -= opti_.skewVR * AngMomentum;
    opti_.vt.noalias() -= robotMass_* (kine::skewSymmetric(position) *
                        (opti_.wx2Rc+ opti_._2wxRv + opti_.Ra ));
    opti_.vt.noalias() -= robotMass_* kine::skewSymmetric(opti_.Rcp) * cst::gravity;

    angularAcceleration = ( opti_.RIRT + robotMass_*kine::skewSymmetric2(opti_.Rc)).llt().solve(
                            (opti_.vt - robotMass_*kine::skewSymmetric(opti_.Rcp)*opti_.vf));

    linearAcceleration = opti_.vf;
    linearAcceleration += kine::skewSymmetric(opti_.Rc)*angularAcceleration;

}

Matrix3& NonLinearRotationalTableCartDevice::computeRotation_
            (const Vector3 & x, int i)
{
    Vector3 & oriV = opti_.orientationVector(i);
    Matrix3 & oriR = opti_.curRotation(i);
    if (oriV!=x)
    {
        oriV = x;
        oriR = kine::rotationVectorToAngleAxis(x).toRotationMatrix();
    }

    return oriR;
}

void NonLinearRotationalTableCartDevice::computeElastContactForcesAndMoments
                          (const IndexedMatrixArray& contactPosArray,
                           const IndexedMatrixArray& contactOriArray,
                           const Vector3& position, const Vector3& linVelocity,
                           const Vector3& oriVector, const Matrix3& orientation,
                           const Vector3& angVel,
                           Vector3& forces, Vector3& moments)
{
    unsigned nbContacts=contactsNumber_;
    fc_.resize(nbContacts*3);
    tc_.resize(nbContacts*3);
    forces.setZero();
    moments.setZero();

    stateObservation::Matrix forcesOut;
    forcesOut.resize(6,2);
    forcesOut.setZero();

  for (unsigned i = 0; i<nbContacts ; ++i)
  {
    opti_.contactPos = contactPosArray[i];

    opti_.Rci = computeRotation_(contactOriArray[i],i+2);
    opti_.Rcit.noalias()= opti_.Rci.transpose();

    opti_.RciContactPos.noalias()= orientation*opti_.contactPos;

    opti_.globalContactPos = position;
    opti_.globalContactPos.noalias() += opti_.RciContactPos ;

    opti_.forcei.noalias() = - opti_.Rci*Kfe_*opti_.Rcit*(opti_.globalContactPos-opti_.contactPos);
    opti_.forcei.noalias() += - opti_.Rci*Kfv_*opti_.Rcit*(kine::skewSymmetric(angVel)*opti_.RciContactPos
                              +linVelocity);

    fc_.segment<3>(3*i)= opti_.forcei;

    forces += opti_.forcei;

    opti_.momenti.noalias() = -opti_.Rci*Kte_*opti_.Rcit*oriVector;
    opti_.momenti.noalias() += -opti_.Rci*Ktv_*opti_.Rcit*angVel;

    tc_.segment<3>(3*i)= opti_.momenti;

    moments.noalias() += opti_.momenti + kine::skewSymmetric(opti_.globalContactPos)*opti_.forcei;

    forcesOut.block(0,i,6,1) <<  fc_.segment<3>(3*i),
                                 tc_.segment<3>(3*i);
    }

    stateObservation::Vector6 force1, force2;
    force1=forcesOut.block(0,0,6,1);
    force2=forcesOut.block(0,1,6,1);
    contact1ForcesSOUT_.setConstant(convertVector<dynamicgraph::Vector>(force1));
    contact2ForcesSOUT_.setConstant(convertVector<dynamicgraph::Vector>(force2));

    sumMomentsSOUT_.setConstant(convertVector<dynamicgraph::Vector>(moments));

}

stateObservation::Vector NonLinearRotationalTableCartDevice::computeDynamics(
                                  double inTimeStep,
                                  stateObservation::Vector &xn,
                                  stateObservation::Vector &un)
{
  double dt = inTimeStep;

  Vector3 linearAcceleration;
  Vector3 angularAcceleration;
  linearAcceleration.setZero();
  angularAcceleration.setZero();
  Vector3 positionCom=xn.block(0,0,3,1);
  Vector3 velocityCom=xn.block(12,0,3,1);
  Vector3 accelerationCom=un.block(0,0,3,1);
  Vector3 AngMomentum;
  Vector3 dotAngMomentum;
  Matrix3 Inertia;
  Matrix3 dotInertia;
  Vector3 position=xn.block(9,0,3,1);
  Vector3 linVelocity=xn.block(21,0,3,1);
  Vector3 oriVector=xn.block(6,0,3,1);
  Matrix3 orientation;
  stateObservation::Vector oriVect;
  oriVect.resize(6);
  oriVect <<    0,
                0,
                0,
                oriVector;
  orientation=(kine::vector6ToHomogeneousMatrix(oriVect)).block(0,0,3,3);
  Vector3 angularVel=xn.block(18,0,3,1);
  Vector6 waistOriVect;
  waistOriVect <<   xn.block(3,0,3,1),
                    0,
                    0,
                    0;
  Matrix3 waistOri=(kine::vector6ToHomogeneousMatrix(waistOriVect)).block(0,0,3,3);
  Vector3 waistAngVel=xn.block(15,0,3,1);
  Vector3 waistAngAcc;
  waistAngAcc   <<  un.block(3,0,2,1),
                    0;
  Inertia=waistOri*I_*waistOri.transpose()-robotMass_*kine::skewSymmetric2(positionCom);
  dotInertia=kine::skewSymmetric(waistAngVel)*waistOri*I_*waistOri.transpose()-waistOri*I_*waistOri.transpose()*kine::skewSymmetric(waistAngVel)
             -robotMass_*kine::skewSymmetric(velocityCom)*kine::skewSymmetric(positionCom)-robotMass_*kine::skewSymmetric(positionCom)*kine::skewSymmetric(velocityCom);
  AngMomentum_=waistOri*I_*waistOri.transpose()*waistAngVel+robotMass_*kine::skewSymmetric(positionCom)*velocityCom;
  dotAngMomentum=waistOri*I_*waistOri.transpose()*waistAngAcc+kine::skewSymmetric(waistAngVel)*waistOri*I_*waistOri.transpose()*waistAngVel
                 +robotMass_*kine::skewSymmetric(positionCom)*velocityCom;

  inertiaSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(Inertia));
  dotInertiaSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(dotInertia));
  angularMomentumSOUT_.setConstant(convertVector<dynamicgraph::Vector>(AngMomentum));
  dotAngularMomentumSOUT_.setConstant(convertVector<dynamicgraph::Vector>(dotAngMomentum));

  computeAccelerations
     (positionCom, velocityCom,
      accelerationCom, AngMomentum,
      dotAngMomentum,
      Inertia, dotInertia,
      opti_.contactPosV,
      opti_.contactOriV,
      position, linVelocity, linearAcceleration,
      oriVector , orientation,
      angularVel, angularAcceleration);

  stateObservation::Vector xn1;
  xn1.resize(24);

  xn1   <<  velocityCom,
            waistAngVel,
            angularVel,
            linVelocity,
            accelerationCom,
            waistAngAcc,
            angularAcceleration,
            linearAcceleration;

  //const dynamicgraph::Vector& k2 = A_*(xn + k1*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k3 = A_*(xn + k2*(dt/2)) + B_*inControl;
  //const dynamicgraph::Vector& k4 = A_*(xn + k3*dt) + B_*inControl;
  //dynamicgraph::Vector dx = (k1 + k2*2 + k3*2 + k4)*(dt/6.);//Runge Kutta 4

  stateObservation::Vector dx = xn1* dt;
  stateObservation::Vector nextState = xn + dx;

  flexAngAccVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(angularAcceleration));
  flexLinAccSOUT_.setConstant(convertVector<dynamicgraph::Vector>(linearAcceleration));

  return nextState;
}

void NonLinearRotationalTableCartDevice::incr(double inTimeStep)
{
  int t = stateSOUT_.getTime();

  // Retrieve control
  stateObservation::Vector un = convertVector<stateObservation::Vector>(controlSIN_ (t));

  // Compute dynamics
  stateObservation::Vector nextState = computeDynamics(inTimeStep,xn_, un);
  xn_=nextState;

  // Rename data
  stateObservation::Vector3 com=xn_.block(0,0,3,1);
  stateObservation::Vector3 waistOriVect=xn_.block(3,0,3,1);
  Matrix3 waistOri=kine::rotationVectorToRotationMatrix(waistOriVect);
  stateObservation::Vector3 flexOriVect=xn_.block(6,0,3,1);
  stateObservation::Vector3 tflex=xn_.block(9,0,3,1);
  Matrix3 flexOri=kine::rotationVectorToRotationMatrix(flexOriVect);
  stateObservation::Vector3 dcom=xn_.block(12,0,3,1);
  stateObservation::Vector3 waistAngVel=xn_.block(15,0,3,1);
  stateObservation::Vector3 flexAngVel=xn_.block(18,0,3,1);
  stateObservation::Vector3 dtflex=xn_.block(21,0,3,1);

  // Waist homo
  Matrix4 waistHomo;
  waistHomo.setZero();
  waistHomo.block(0,0,3,3)=waistOri;

  // Control state
  stateObservation::Vector nextControlState;
  nextControlState.resize(18);
  nextControlState  <<  com,
                        waistOriVect,
                        flexOriVect,
                        dcom,
                        waistAngVel,
                        flexAngVel;

  // Output signals
  stateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextState));
  stateSOUT_.setTime (t+1);
  controlStateSOUT_.setConstant(convertVector<dynamicgraph::Vector>(nextControlState));
  controlStateSOUT_.setTime(t+1);
  comSOUT_.setConstant(convertVector<dynamicgraph::Vector>(com));
  comSOUT_.setTime(t+1);
  waistHomoSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(waistHomo));
  waistHomoSOUT_.setTime(t+1);
  flexOriVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(flexOriVect));
  flexOriVectSOUT_.setTime(t+1);
  comDotSOUT_.setConstant(convertVector<dynamicgraph::Vector>(dcom));
  comDotSOUT_.setTime(t+1);
  stateObservation::Vector6 waistAngVelOut; waistAngVelOut.setZero();
  waistAngVelOut    <<  0,
                        0,
                        0,
                        waistAngVel;
  waistAngVelSOUT_.setConstant(convertVector<dynamicgraph::Vector>(waistAngVelOut));
  waistAngVelSOUT_.setTime(t+1);
  flexAngVelVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(flexAngVel));
  flexAngVelVectSOUT_.setTime(t+1);

  /// Post treatments
  // World state
  stateObservation::Vector xnWorld;
  xnWorld.resize(24);
  xnWorld <<  flexOri*com+tflex,
              kine::rotationMatrixToRotationVector(flexOri*waistOri),
              flexOriVect,
              tflex,
              kine::skewSymmetric(flexAngVel)*flexOri*com+flexOri*dcom+dtflex,
              flexAngVel+flexOri*waistAngVel,
              flexAngVel,
              dtflex;
  stateWorldSOUT_.setConstant (convertVector<dynamicgraph::Vector>(xnWorld));


  // Energy
  double Etot, Eflex, Ecom, Ewaist;
  Eflex=0.5*Kte_(0,0)*(flexOriVect).squaredNorm();
  Ecom=0.5*robotMass_*(xnWorld.block(12,0,3,1)).squaredNorm();
  Ewaist=0.5*AngMomentum_.dot(waistAngVel);
  Etot=Eflex+Ecom+Ewaist;
  stateObservation::Vector energy;
  energy.resize(4);
  energy <<   Etot,
              Ecom,
              Ewaist,
              Eflex;
  energySOUT_.setConstant(convertVector<dynamicgraph::Vector>(energy));
}

}
