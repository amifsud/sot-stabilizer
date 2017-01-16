//
// Copyright (c) 2017,
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

    /// Signals

        /// State
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotSIN_;

        /// Reference state
    // Position of center of mass
    SignalPtr < dynamicgraph::Vector, int> comRefSIN_;
    // Velocity of center of mass
    SignalPtr < dynamicgraph::Vector, int> comDotRefSIN_;

        /// Jacobians
    SignalPtr < dynamicgraph::Matrix, int> jacobianComSIN_;

        /// Control gain
    SignalPtr <double, int> controlGainSIN_;

        /// Outputs
    // state output
    SignalTimeDependent <Vector, int> stateSOUT_;
    // state reference output
    SignalTimeDependent <Vector, int> stateRefSOUT_;
    // state error output
    SignalTimeDependent <Vector, int> stateErrorSOUT_;
    // control output
    SignalTimeDependent <Vector, int> controlSOUT_;

    /// Parameters
    // Time sampling period
    double dt_;
    // Whether stabilizer is on
    bool on_;

    controller::DiscreteTimeOrdinarySMC controller_;

    stateObservation::Vector xk_;
    stateObservation::Vector xkRef_;
    stateObservation::Vector dxk_;
    stateObservation::Vector uk_;
    stateObservation::Vector preTask_;

  }; // class Stabilizer
} // namespace sotStabiilizer

#endif // SOT_DYNAMIC_STABILIZER_HRP2_TWODOF_LQR_HH

