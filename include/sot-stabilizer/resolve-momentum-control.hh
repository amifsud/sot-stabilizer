//
// Copyright (c) 2016,
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

#ifndef RESOLVE_MOMENTUM_CONTROL_HH
# define RESOLVE_MOMENTUM_CONTROL_HH

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


#include <sot-state-observation/tools/definitions.hh>


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
  /// This task takes as input the following signals
  /// and provides as output the following signals
  /// \li taskSOUT, the desired task,
  /// \li jacobianSOUT, the jacobian of the desired task
  class ResolveMomentumControl : public TaskAbstract
  {
    DYNAMIC_GRAPH_ENTITY_DECL ();
  public:

    /// Constructor by name
    ResolveMomentumControl(const std::string& inName);
    ~ResolveMomentumControl() {}

    /// Documentation of the entity
    virtual std::string getDocString () const
    {
      std::string doc =
        "Resolve momentum control stabilizerr\n";
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


  private:

    /// Methods
    VectorMultiBound&  computeTask(VectorMultiBound& task, const int& time);
    Matrix& computeJacobian(Matrix& jacobian, const int& time);

    /// Signals

    SignalPtr < dynamicgraph::Vector, int> zmpRefSIN_;
    SignalPtr < dynamicgraph::Vector, int> momentaSIN_;
    SignalPtr < dynamicgraph::Matrix, int> inertiaSIN_;
    SignalPtr < dynamicgraph::Matrix, int> dinertiaSIN_;
    SignalPtr < dynamicgraph::Vector, int> estimatorStateSIN_;
    SignalPtr < dynamicgraph::Vector, int> estimatorInputSIN_;

        /// Outputs

    /// Parameters

    bool on_;
    double dt_;


  }; // class ResolveMomentumControl
} // namespace sotStabiilizer

#endif // RESOLVE_MOMENTUM_CONTROL_HH

