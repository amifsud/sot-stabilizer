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

#ifndef SOT_DYNAMIC_STABILIZER_HRP2_SMC_HH
# define SOT_DYNAMIC_STABILIZER_HRP2_SMC_HH

#include <dynamic-graph/linear-algebra.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal.h>
#include <dynamic-graph/signal-ptr.h>

#include <sot/core/task-abstract.hh>
#include <sot/core/matrix-rotation.hh>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/multi-bound.hh>
#include <sot/core/vector-utheta.hh>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <sot-stabilizer/controllers/discrete-time-ordinary-smc.hh>
#include <sot-stabilizer/controllers/discrete-time-lti-lqr.hh>
#include <sot-state-observation/tools/definitions.hh>

//#define NDEBUG
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
  using dynamicgraph::sot::MatrixHomogeneous;
  using dynamicgraph::sot::MatrixRotation;
  using dynamicgraph::sot::VectorUTheta;
  using dynamicgraph::sot::VectorRollPitchYaw;

  using namespace sotStateObservation;
  using namespace stateObservation;

/// Dynamic balance stabilizer
///
/// This task takes as input four signals
/// \li comSIN, the position of the center of mass (COM)
/// \li comDesSIN, the the desired position of the center of mass,
/// \li zmpSIN, the position of the center of pressure (ZMP),
/// \li zmpDesSIN, the desired position of the center of pressure,
/// \li jacobianSIN, the jacobian of the center of mass,
/// \li comdotSIN, reference velocity of the center of mass,
/// and provides as output two signals
/// \li taskSOUT, the desired time derivative of the center of mass,
/// \li jacobianSOUT, the jacobian of the center of mass
  class HRP2SMCStabilizer : public TaskAbstract
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:
    // Constant values
    static double constm_;

    /// Constructor by name
    HRP2SMCStabilizer(const std::string& inName);
    ~HRP2SMCStabilizer() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
      std::string doc =
        "Dynamic balance humanoid robot stabilizer\n"
        "\n"
        "This task aims at controlling balance for a walking legged humanoid"
        " robot.\n"
        "The entity takes 6 signals as input:\n"
        "  - deltaCom: the difference between the position of the center of "
        "mass and the\n"
        " reference,\n"
        "  - Jcom: the Jacobian of the center of mass wrt the robot "
        "configuration,\n"
        "  - comdot: the reference velocity of the center of mass \n"
        "  \n"
        "As any task, the entity provide two output signals:\n"
        "  - task: the velocity of the center of mass so as to cope with\n"
        "          perturbations,\n"
        "  - jacobian: the Jacobian of the center of mass with respect to "
        "robot\n"
        "              configuration.\n";
      return doc;
    }

    /// Start stabilizer
    void start ()
    {
      on_ = true;

      //flexOriRefSIN_.setConstant(flexOriVectSIN_);
    }

    /// Start stabilizer
    void stop ()
    {
      on_ = false;
    }

    /// @}
    /// \name Sampling time period
    /// @{

    /// \brief Set sampling time period
    void setTimePeriod(const double& inTimePeriod)
    {
      dt_ = inTimePeriod;
    }
    /// \brief Get sampling time period
    double getTimePeriod() const
    {
      return dt_;
    }
    /// @}


    Vector& getControl(Vector& control, const int& time);

  private:

    /// Methods
    // Compute the task
    VectorMultiBound& computeControlFeedback(VectorMultiBound& comdot, const int& time);
    // Compute the jacobian
    Matrix& computeJacobian(Matrix& jacobian, const int& time);
    // Compute the dynamics matrices
    void computeDynamicsMatrix(const stateObservation::Vector3 cl, const stateObservation::Matrix Kth, const stateObservation::Matrix Kdth, const int& time);
    // Compute the inertia tensor
    stateObservation::Matrix3 computeInert(const stateObservation::Vector& com, const int& time);


    /// Signals
        /// State
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    // Homogeneous representation of the waist position
    SignalPtr <dynamicgraph::Matrix, int> waistHomoSIN_;
    // Orientation of the flexibility on vector form
    SignalPtr <dynamicgraph::Vector, int> flexOriVectSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotSIN_;
    // Refrence angular velocity of the waist
    SignalPtr < dynamicgraph::Vector , int > waistAngVelSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> flexAngVelVectSIN_;

        /// Bias on the CoM
    SignalPtr < dynamicgraph::Vector, int> comBiasSIN_;

        /// Reference state
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefSIN_;
    // Perturbation
    SignalPtr < dynamicgraph::Vector, int> perturbationVelSIN_;
    SignalPtr < dynamicgraph::Vector, int> perturbationAccSIN_;
    // Homogeneous representation of the waist position
    SignalPtr <dynamicgraph::Vector, int> waistOriRefSIN_;
    // Orientation of the flexibility on vector form
    SignalPtr <dynamicgraph::Vector, int> flexOriRefSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotRefSIN_;
    // Refrence angular velocity of the waist
    SignalPtr < dynamicgraph::Vector , int > waistVelRefSIN_;
    // Velocity of the flexibility
    SignalPtr <dynamicgraph::Vector, int> flexAngVelRefSIN_;

        /// Support contact positions
    SignalPtr <dynamicgraph::Vector, int> supportPos1SIN_;
    SignalPtr <dynamicgraph::Vector, int> supportPos2SIN_;

        /// Inertia
    SignalPtr < dynamicgraph::Matrix, int> inertiaSIN_;

        /// Jacobians
    SignalPtr < dynamicgraph::Matrix, int> jacobianComSIN_;
    SignalPtr < dynamicgraph::Matrix, int> jacobianWaistSIN_;
    SignalPtr < dynamicgraph::Matrix, int> jacobianChestSIN_;

        /// Control gain for 0 supports case
    SignalPtr <double, int> controlGainSIN_;

        /// Outputs
    // state output
    SignalTimeDependent <Vector, int> stateSOUT_;
    // state reference output
    SignalTimeDependent <Vector, int> stateRefSOUT_;
    // error state output
    SignalTimeDependent <Vector, int> stateErrorSOUT_;
    // error output
    SignalTimeDependent <Vector, int> errorSOUT_;
    // control output
    SignalTimeDependent <Vector, int> controlSOUT_;
    // integrated task output
    SignalTimeDependent <Vector, int> integratedTaskSOUT_;

    // Number of support feet
    SignalPtr <unsigned int, int> nbSupportSIN_;


    /// Parameters
    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;
    // Number of feet in support
    unsigned int nbSupport_;
    Vector supportPos1_, supportPos2_;

    controller::DiscreteTimeOrdinarySMC SMCController_;
    controller::DiscreteTimeLTILQR LQRController_;

    stateObservation::Vector preTask_;
    stateObservation::Vector integratedTask_;

    // For the LQR
    bool computed_;
    bool fixedGains_;
    stateObservation::Matrix3 Kth_, Kdth_;
    double ktd_, kts_, kfs_, kfd_;
    stateObservation::Matrix A_, B_;
    stateObservation::Matrix3 I_;

    double hrp2Mass_;

  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH

