/*
 * Copyright 2015,
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

#ifndef DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH
#define DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

#include <state-observation/tools/definitions.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot/core/matrix-homogeneous.hh>

namespace sotStabilizer {

    using namespace sotStateObservation;
    using namespace stateObservation;

    /// \brief ROtational table cart model
    ///
    /// This class represents the classical table-cart model as used as a
    /// simplified model for a humanoid robot but with a one more degree of
    /// freedom allowding the control of the orientation of a body around the com
    ///
    class NonLinearRotationalTableCartDevice : public dynamicgraph::Entity
    {
      DYNAMIC_GRAPH_ENTITY_DECL ();
    public:
      /**
         \brief Constructor by name
      */
      NonLinearRotationalTableCartDevice(const std::string& inName);

      ~NonLinearRotationalTableCartDevice();

      /// Integrate equation of motion over time step given as input
      void incr(double inTimeStep);

      /// \name Parameters
      /// @{

      /// \brief Set the mass of the cart
      void setRobotMass (const double& inMass) {
        robotMass_ = inMass;
        robotMassInv_ = 1/inMass;
      }

      void setContactPosition(unsigned i, const Matrix4 & positionHomo);
      Vector6 getContactPosition(unsigned i);

      struct Optimization
      {


        Vector3 positionFlex;
        Vector3 velocityFlex;
        Vector3 accelerationFlex;
        Vector3 orientationFlexV;
        Vector3 angularVelocityFlex;
        Vector3 angularAccelerationFlex;

        Matrix3 rFlex;
        Matrix3 rFlexT;

        AngleAxis orientationAA;

        Vector xk1;

        Vector3 positionCom;
        Vector3 velocityCom;
        Vector3 accelerationCom;
        Vector3 AngMomentum;
        Vector3 dotAngMomentum;

        Vector3 positionControl;
        Vector3 velocityControl;
        Vector3 accelerationControl;
        Vector3 orientationControlV;
        Vector3 angularVelocityControl;

        Matrix3 rControl;

        Matrix3 rimu;

        IndexedMatrixArray contactPosV;
        IndexedMatrixArray contactOriV;


        Matrix3 inertia;
        Matrix3 dotInertia;

        Vector3 fc;
        Vector3 tc;

        Vector3 vf;
        Vector3 vt;

        //elastic contact forces and moments
        Matrix3 Rci; //rotation of contact i
        Matrix3 Rcit;//transpose of previous
        Vector3 contactPos; //
        Vector3 RciContactPos;
        Vector3 globalContactPos;

        Vector3 forcei;
        Vector3 momenti;


        Matrix3 skewV;
        Matrix3 skewV2;
        Matrix3 skewVR;
        Matrix3 skewV2R;
        Matrix3 RIRT;
        Vector3 wx2Rc;
        Vector3 _2wxRv;
        Vector3 Ra;
        Vector3 Rc;
        Vector3 Rcp;

        //optimization of orientation transformation between vector3 to rotation matrix

        Matrix3 curRotation0;
        Vector3 orientationVector0;
        Matrix3 curRotation1;
        Vector3 orientationVector1;
        Matrix3 curRotation2;
        Vector3 orientationVector2;
        Matrix3 curRotation3;
        Vector3 orientationVector3;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Optimization()
          :
          curRotation0(Matrix3::Identity()),
          orientationVector0(Vector3::Zero()),
          curRotation1(Matrix3::Identity()),
          orientationVector1(Vector3::Zero()),
          curRotation2(Matrix3::Identity()),
          orientationVector2(Vector3::Zero()),
          curRotation3(Matrix3::Identity()),
          orientationVector3(Vector3::Zero())
        {}

        inline Vector3& orientationVector(int i)
        {
          if (i==0)
            return orientationVector0;
          if (i==1)
            return orientationVector1;
          if (i==2)
            return orientationVector2;

          return orientationVector3;
        }

        inline Matrix3& curRotation(int i)
        {
          if (i==0)
            return curRotation0;
          if (i==1)
            return curRotation1;
          if (i==2)
            return curRotation2;

          return curRotation3;
        }
      } opti_;


      void computeElastContactForcesAndMoments
                                (const IndexedMatrixArray& contactPosArray,
                                 const IndexedMatrixArray& contactOriArray,
                                 const Vector3& position, const Vector3& linVelocity,
                                 const Vector3& oriVector, const Matrix3& orientation,
                                 const Vector3& angVel,
                                 Vector3& forces, Vector3& moments);

      void computeAccelerations
         (const Vector3& positionCom, const Vector3& velocityCom,
          const Vector3& accelerationCom, const Vector3& AngMomentum,
          const Vector3& dotAngMomentum,
          const Matrix3& Inertia, const Matrix3& dotInertia,
          const IndexedMatrixArray& contactPosArray,
          const IndexedMatrixArray& contactOriV,
          const Vector3& position, const Vector3& linVelocity, Vector3& linearAcceleration,
          const Vector3 &oriVector ,const Matrix3& orientation,
          const Vector3& angularVel, Vector3& angularAcceleration);

      /// \brief Get the mass of the cart
      double getRobotMass () const {
        return robotMass_;
      }

      void setMomentOfInertia(const dynamicgraph::Matrix& momentOfInertia)
      {
          I_=convertMatrix<stateObservation::Matrix>(momentOfInertia);
      }

      dynamicgraph::Matrix getMomentOfInertia() const
      {
          return convertMatrix<dynamicgraph::Matrix>(I_);
      }

      /// \brief Set the stiffness of the flexibility
      void setKfe (const dynamicgraph::Matrix& inStiffness) {
          Kfe_ = convertMatrix<stateObservation::Matrix>(inStiffness);
      }

      /// \brief Get the stiffness of the flexibility
      dynamicgraph::Matrix getKfe () const {
        return convertMatrix<dynamicgraph::Matrix>(Kfe_);
      }

      /// \brief Set the viscosity of the flexibility
      void setKfv (const dynamicgraph::Matrix& inViscosity) {
        Kfv_ = convertMatrix<stateObservation::Matrix>(inViscosity);
      }

      /// \brief Get the viscosity of the flexibility
      dynamicgraph::Matrix getKfv () const {
        return convertMatrix<dynamicgraph::Matrix>(Kfv_);
      }

      /// \brief Set the stiffness of the flexibility
      void setKte (const dynamicgraph::Matrix& inStiffness) {
          Kte_ = convertMatrix<stateObservation::Matrix>(inStiffness);
      }

      /// \brief Get the stiffness of the flexibility
      dynamicgraph::Matrix getKte () const {
        return convertMatrix<dynamicgraph::Matrix>(Kte_);
      }

      /// \brief Set the viscosity of the flexibility
      void setKtv (const dynamicgraph::Matrix& inViscosity) {
        Ktv_ = convertMatrix<stateObservation::Matrix>(inViscosity);
      }

      /// \brief Get the viscosity of the flexibility
      dynamicgraph::Matrix getKtv () const {
        return convertMatrix<dynamicgraph::Matrix>(Ktv_);
      }

      void setContactsNumber(unsigned int nb)
      {
          contactsNumber_= nb;
      }

      void setState(const dynamicgraph::Vector& inState)
      {
          controlStateSOUT_.setConstant(inState);
          stateObservation::Vector state;
          state=convertVector<stateObservation::Vector>(inState);
          xn_.resize(state.size());
          xn_=state;
          comSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state.block(0,0,3,1)));
          Matrix3 waistOri=kine::rotationVectorToRotationMatrix(state.block(3,0,3,1));
          Matrix4 waistHomo;
          waistHomo.setZero();
          waistHomo.block(0,0,3,3)=waistOri;
          waistHomoSOUT_.setConstant(convertMatrix<dynamicgraph::Matrix>(waistHomo));
          flexOriVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state.block(6,0,3,1)));
          comDotSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state.block(12,0,3,1)));
          waistAngVelSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state.block(15,0,3,1)));
          flexAngVelVectSOUT_.setConstant(convertVector<dynamicgraph::Vector>(state.block(18,0,3,1)));
      }

      stateObservation::Vector & getState()
      {
          return xn_;
      }

      inline Matrix3& computeRotation_(const Vector3 & x, int i);
      /**
         @}
      */

      /**
         \brief Compute the evolution of the state of the pendulum
      */
      void computeDynamics(
              double inTimeStep,
              stateObservation::Vector &un);

    private:
      /// Control
      dynamicgraph::SignalPtr< dynamicgraph::Vector, int > controlSIN_;

      /// State of the table cart
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> stateSOUT_;
      /// Control state of the table cart
      dynamicgraph::Signal< ::dynamicgraph::Vector, int> controlStateSOUT_;

      /// State
      // Position of center of mass
      dynamicgraph::SignalPtr < dynamicgraph::Vector, int> comSOUT_;
      // Homogeneous representation of the waist position
      dynamicgraph::SignalPtr <dynamicgraph::Matrix, int> waistHomoSOUT_;
      // Orientation of the flexibility on vector form
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> flexOriVectSOUT_;
      // Velocity of center of mass
      dynamicgraph::SignalPtr < dynamicgraph::Vector, int> comDotSOUT_;
      // Refrence angular velocity of the waist
      dynamicgraph::SignalPtr < dynamicgraph::Vector , int > waistAngVelSOUT_;
      // Velocity of the flexibility
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> flexAngVelVectSOUT_;

      // Angular acceleration of the flexibility
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> flexAngAccVectSOUT_;
      // Linear acceleration of the flexibility
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> flexLinAccSOUT_;

      // contacts positions and forces
      dynamicgraph::SignalPtr <dynamicgraph::sot::MatrixHomogeneous, int> contact1PosSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> contact1ForcesSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::sot::MatrixHomogeneous, int> contact2PosSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> contact2ForcesSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> sumMomentsSOUT_;

      // Inertial values
      dynamicgraph::SignalPtr <dynamicgraph::Matrix, int> inertiaSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Matrix, int> dotInertiaSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> angularMomentumSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> dotAngularMomentumSOUT_;

      // Post treaments
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> stateWorldSOUT_;
      dynamicgraph::SignalPtr <dynamicgraph::Vector, int> energySOUT_;

      /// \brief Mass of the cart
      double robotMass_;
      double robotMassInv_;
      /// \brief Height of the cart
      stateObservation::Vector cl_;
      /// \brief Stiffness of the flexibility
      stateObservation::Matrix Kfe_;
      /// \brief Viscosity coefficient
      stateObservation::Matrix Kfv_;
      /// \brief Stiffness of the flexibility
      stateObservation::Matrix Kte_;
      /// \brief Viscosity coefficient
      stateObservation::Matrix Ktv_;
      /// Moment of inertia around y axis
      stateObservation::Matrix I_;

      Vector fc_;
      Vector tc_;

      stateObservation::Vector xn_;

      stateObservation::Vector AngMomentum_;
      
      stateObservation::Matrix4 supportPos1_;
      stateObservation::Matrix4 supportPos2_;

      unsigned int contactsNumber_;
      std::vector <Vector3,Eigen::aligned_allocator<Vector3> > contactPositions_;

    };
}
#endif // DYNAMIC_GRAPH_ROTATIONAL_TABLE_CART_DEVICE_HH
