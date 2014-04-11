//
// Copyright (c) 2012,
// Florent Lamiraux
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

#include <sot-stabilizer/hrp2-decoupled-stabilizer.hh>

namespace sot
{
namespace dynamic
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

double HRP2DecoupledStabilizer::m_ = 59.8;
double HRP2DecoupledStabilizer::g_ = 9.81;
double HRP2DecoupledStabilizer::zeta_ = .80;

DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN (HRP2DecoupledStabilizer, "HRP2DecoupledStabilizer");

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
HRP2DecoupledStabilizer::HRP2DecoupledStabilizer(const std::string& inName) :
    TaskAbstract(inName),
    comSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::com"),
    comRefSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::comRef"),
    jacobianSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(matrix)::Jcom"),
    comdotSIN_ (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::comdot"),
    leftFootPositionSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::leftFootPosition"),
    rightFootPositionSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::rightFootPosition"),
    forceLeftFootSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::force_lf"),
    forceRightFootSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::force_rf"),
    stateFlexSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(MatrixHomo)::stateFlex"),
    stateFlexDotSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(vector)::stateFlexDot"),
    controlGainSIN_
    (NULL, "HRP2DecoupledStabilizer("+inName+")::input(double)::controlGain"),
    d2comSOUT_ ("HRP2DecoupledStabilizer("+inName+")::output(vector)::d2com"),
    nbSupportSOUT_
    ("HRP2DecoupledStabilizer("+inName+")::output(unsigned)::nbSupport"),
    errorSOUT_ ("HRP2DecoupledStabilizer("+inName+")::output(vector)::error"),
    supportPos1SOUT_("HRP2DecoupledStabilizer("+inName+")::output(vector)::supportPos1"),
    supportPos2SOUT_("HRP2DecoupledStabilizer("+inName+")::output(vector)::supportPos2"),
    gain1_ (4), gain2_ (4), gainz_ (4), gainLat_ (4),
    prevCom_(3), dcom_ (3), dt_ (.005), on_ (false),
    forceThreshold_ (.036*m_*g_), angularStiffness_ (425.), d2com_ (3),
    deltaCom_ (3),
    flexPosition_ (), flexPositionLf_ (), flexPositionRf_ (),
    flexPositionLat_ (),
    flexVelocity_ (6), flexVelocityLf_ (6), flexVelocityRf_ (6),
    flexVelocityLat_ (6),
    flexZobs_ (2), flexLatControl_ (1), flexLatObs_ (2),
    timeBeforeFlyingFootCorrection_ (.1),
    iterationsSinceLastSupportLf_ (0), iterationsSinceLastSupportRf_ (0),
    supportCandidateLf_ (0), supportCandidateRf_ (0),
    uth_ (),
    R_ (), translation_ (3), zmp_ (3),
    theta1Ref_ (0), theta1RefPrev_ (0), dtheta1Ref_ (0),
    debug_ (4)
{
    // Register signals into the entity.
    signalRegistration (comSIN_);
    signalRegistration (comRefSIN_);
    signalRegistration (jacobianSIN_);
    signalRegistration (comdotSIN_);
    signalRegistration (leftFootPositionSIN_ << rightFootPositionSIN_
                        << forceRightFootSIN_ << forceLeftFootSIN_);
    signalRegistration (stateFlexSIN_ << stateFlexDotSIN_);
    signalRegistration (controlGainSIN_);
    signalRegistration (d2comSOUT_);
    signalRegistration (nbSupportSOUT_ << supportPos1SOUT_ << supportPos2SOUT_);
    signalRegistration (errorSOUT_);

    taskSOUT.addDependency (comSIN_);
    taskSOUT.addDependency (comRefSIN_);
    taskSOUT.addDependency (comdotSIN_);
    taskSOUT.addDependency (leftFootPositionSIN_);
    taskSOUT.addDependency (rightFootPositionSIN_);
    taskSOUT.addDependency (forceRightFootSIN_);
    taskSOUT.addDependency (forceLeftFootSIN_);
    taskSOUT.addDependency (stateFlexSIN_);
    taskSOUT.addDependency (stateFlexDotSIN_);
    taskSOUT.addDependency (controlGainSIN_);

    jacobianSOUT.addDependency (jacobianSIN_);

    taskSOUT.setFunction (boost::bind(&HRP2DecoupledStabilizer::computeControlFeedback,
                                      this,_1,_2));
    jacobianSOUT.setFunction (boost::bind(&HRP2DecoupledStabilizer::computeJacobianCom,
                                          this,_1,_2));
    nbSupportSOUT_.addDependency (taskSOUT);

    d2com_.setZero ();
    dcom_.setZero ();
    deltaCom_.setZero ();
    d2comSOUT_.setConstant (d2com_);
    flexVelocity_.setZero ();
    flexVelocityLat_.setZero ();

    std::string docstring;
    docstring =
        "\n"
        "    Set sampling time period task\n"
        "\n"
        "      input:\n"
        "        a floating point number\n"
        "\n";
    addCommand("setTimePeriod",
               new dynamicgraph::command::Setter<HRP2DecoupledStabilizer, double>
               (*this, &HRP2DecoupledStabilizer::setTimePeriod, docstring));
    docstring =
        "\n"
        "    Get sampling time period task\n"
        "\n"
        "      return:\n"
        "        a floating point number\n"
        "\n";
    addCommand("getTimePeriod",
               new dynamicgraph::command::Getter<HRP2DecoupledStabilizer, double>
               (*this, &HRP2DecoupledStabilizer::getTimePeriod, docstring));

    addCommand ("start",
                makeCommandVoid0 (*this, &HRP2DecoupledStabilizer::start,
                                  docCommandVoid0 ("Start stabilizer")));

    addCommand ("setGain1",
                makeDirectSetter (*this, &gain1_,
                                  docDirectSetter
                                  ("Set gains single support",
                                   "vector")));

    addCommand ("getGain1",
                makeDirectGetter (*this, &gain1_,
                                  docDirectGetter
                                  ("Get gains single support",
                                   "vector")));

    addCommand ("setGain2",
                makeDirectSetter (*this, &gain2_,
                                  docDirectSetter
                                  ("Set gains double support",
                                   "vector")));

    addCommand ("getGain2",
                makeDirectGetter (*this, &gain2_,
                                  docDirectGetter
                                  ("Get gains double support",
                                   "vector")));

    addCommand ("setGainz",
                makeDirectSetter (*this, &gainz_,
                                  docDirectSetter
                                  ("Set gains of vertical flexibility",
                                   "vector")));

    addCommand ("getGainz",
                makeDirectGetter (*this, &gainz_,
                                  docDirectGetter
                                  ("Get gains of vertical flexibility",
                                   "vector")));

    addCommand ("setGainLateral",
                makeDirectSetter (*this, &gainLat_,
                                  docDirectSetter
                                  ("Set gains of lateral flexibility",
                                   "vector")));

    addCommand ("getGainLateral",
                makeDirectGetter (*this, &gainLat_,
                                  docDirectGetter
                                  ("Get gains of lateral flexibility",
                                   "vector")));

    prevCom_.fill (0.);

    // Single support gains for
    //  - kth = 510,
    //  - zeta = .8
    //  - m = 56
    //  - eigen values = 3, 3, 6, 6.
    gain1_ (0) = 121.27893435294121;
    gain1_ (1) = -19.899754625210093;
    gain1_ (2) = 38.133760000000009;
    gain1_ (3) = -17.707008000000005;

    // Double support gains for
    //  - kth = 2*510,
    //  - zeta = .8
    //  - m = 56
    //  - eigen values = 4, 4, 8, 8.
    gain2_ (0) = 118.62268196078432;
    gain2_ (1) = 58.543997288515406;
    gain2_ (2) = 37.326305882352941;
    gain2_ (3) = -10.661044705882359;

    // Vectical gains for
    //  - kz = 150000
    //  - m = 56
    //  - eigen values = 21, 21, 21, 21.
    gainz_ (0) = 25.23;
    gainz_ (1) = -124.77;
    gainz_ (2) = 10.19;
    gainz_ (3) = -9.81;

    // Lateral gains for
    //  - kth = 3000 (kz = 160000, h=.19)
    //  - m = 56
    //  - eigen values = 8, 8, 8, 8.
    gainLat_ (0) = 94.721917866666672;
    gainLat_ (1) = 174.26817999238096;
    gainLat_ (2) = 29.154645333333335;
    gainLat_ (3) = 2.2762837333333308;

    zmp_.setZero ();
}

/// Compute the control law
VectorMultiBound&
HRP2DecoupledStabilizer::computeControlFeedback(VectorMultiBound& comdot,
                                   const int& time)
{
    const Vector deltaCom = comSIN_ (time) - comRefSIN_ (time);
    const Vector& comdotRef = comdotSIN_ (time);
    const MatrixHomogeneous& flexibility =
                stateFlexSIN_.access(time);
    const Vector& flexDot =
                stateFlexDotSIN_.access(time);
    const MatrixHomogeneous& leftFootPosition =
        leftFootPositionSIN_.access (time);
    const MatrixHomogeneous& rightFootPosition =
        rightFootPositionSIN_.access (time);
    const double& gain = controlGainSIN_.access (time);
    const Vector& forceLf = forceLeftFootSIN_.access (time);
    const Vector& forceRf = forceRightFootSIN_.access (time);


    double x = deltaCom_ (0);
    double y = deltaCom_ (1);
    double z = deltaCom_ (2);

    MatrixRotation flexibilityRot;
    flexibility.extract(flexibilityRot);

    // z-component of center of mass deviation in global frame
    flexZobs_ (0) = deltaCom (2);
    flexLatObs_ (0) = 0;

    double theta0, dtheta0;
    double theta1, dtheta1, ddxi;
    double xi, dxi, lat, dlat, ddlat;
    //double thetaz;
    //double dthetaz;
    double fzRef, Zrefx, Zrefy, fz, Zx, Zy;

    flexLatControl_ (0) = 0.;

    //feet position
    Vector rfpos(3);
    Vector lfpos(3);



    // Express vertical component of force in global basis
    double flz = leftFootPosition (2,0) * forceLf (0) +
                 leftFootPosition(2,1) * forceLf (1) +
                 leftFootPosition (2,2) * forceLf (2);
    double frz = rightFootPosition (2,0) * forceRf (0) +
                 rightFootPosition(2,1) * forceRf (1) +
                 rightFootPosition (2,2) * forceRf (2);


    //compute the number of supports
    nbSupport_ = 0;
    if (on_)
    {
        if (frz >= forceThreshold_)
        {
            rightFootPosition.extract(rfpos);
            nbSupport_++;
            supportCandidateRf_++;
            supportPos1SOUT_.setConstant (rfpos);
            nbSupportSOUT_.setTime (time);
            if (supportCandidateRf_ >= 3)
            {
                iterationsSinceLastSupportRf_ = 0;

            }
        }
        else
        {
            supportCandidateRf_ = 0;
            iterationsSinceLastSupportRf_ ++;
        }
        if (flz >= forceThreshold_)
        {
            leftFootPosition.extract(rfpos);
            nbSupport_++;
            supportCandidateLf_++;
            if (nbSupport_==0)
            {
                supportPos1SOUT_.setConstant (rfpos);
                supportPos1SOUT_.setTime (time);
            }
            else
            {
                supportPos2SOUT_.setConstant (rfpos);
                supportPos2SOUT_.setTime (time);
            }

            if (supportCandidateLf_ >= 3)
            {
                iterationsSinceLastSupportLf_ = 0;
            }
        }
        else
        {
            supportCandidateLf_ = 0;
            iterationsSinceLastSupportLf_++;
        }
    }

    switch (nbSupport_)
    {
    case 0:
        dcom_ (0) = -gain * x;
        dcom_ (1) = -gain * y;
        dcom_ (2) = -gain * z;
        break;
    case 1: //single support
    {
        VectorRollPitchYaw rpyFlex;
        rpyFlex.fromMatrix(flexibilityRot);


        //along x
        theta0 = rpyFlex (1);
        dtheta0 = flexDot (1);
        d2com_ (0)= -(gain1_ (0)*x + gain1_ (1)*theta0 +
                      gain1_ (2)*dcom_ (0) + gain1_ (3)*dtheta0);
        dcom_ (0) += dt_ * d2com_ (0);

        // along y
        theta1 = rpyFlex (0);
        dtheta1 = flexDot (0);
        d2com_ (1) = - (gain1_ (0)*y + gain1_ (1)*theta1 +
                        gain1_ (2)*dcom_ (1) + gain1_ (3)*dtheta1);
        dcom_ (1) += dt_ * d2com_ (1);
        // along z
        //d2com_ (2) = - (gainz_ (0)*z + gainz_ (1)*thetaz +
        //		gainz_ (2)*dcom_ (2) + gainz_ (3)*dthetaz);
        //dcom_ (2) += dt_ * d2com_ (2);
    }
        break;
    case 2: //double support
    {
        // compute component of angle orthogonal to the line joining the feet
        double delta_x = lfpos (0) - rfpos (0);
        double delta_y = lfpos (1) - rfpos (1);
        double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);

        u2x_ = delta_x/stepLength;
        u2y_ = delta_y/stepLength;
        u1x_ = u2y_;
        u1y_ = -u2x_;

        VectorRollPitchYaw rpyFlex;
        rpyFlex.fromMatrix(flexibilityRot);

        //along the orthogonal to the contacts line
        theta0 = u1x_ * rpyFlex (1) + u1y_ * rpyFlex (0);
        dtheta0 = u1x_ * flexDot (1) + u1y_ * flexDot (0);
        xi = u1x_*x + u1y_*y;
        dxi = u1x_*dcom_ (0) + u1y_*dcom_ (1);
        ddxi = - (gain2_ (0)*xi + gain2_ (1)*theta0 + gain2_ (2)*dxi +
                  gain2_ (3)*dtheta0);

        //along the contacts line
        theta1 = u2x_ * rpyFlex (1) + u2y_ * rpyFlex (0);
        dtheta1 = u2x_ * flexDot (1) + u2y_ * flexDot (0);
        lat = u2x_*x + u2y_*y;
        dlat = u2x_*dcom_ (0) + u2y_*dcom_ (1);
        ddlat = - (gainLat_ (0)*lat + gainLat_ (1)*(theta1-theta1Ref_)
                   + gainLat_ (2)*dlat + gainLat_ (3)*(dtheta1 - dtheta1Ref_));
        flexLatControl_ (0) = ddlat;
        debug_ (0) = lat;
        debug_ (1) = theta1-theta1Ref_;
        debug_ (2) = dlat;
        debug_ (3) = dtheta1 - dtheta1Ref_;

        d2com_ (0) = ddxi * u1x_ + ddlat*u2x_;
        d2com_ (1) = ddxi * u1y_ + ddlat*u2y_;
        dcom_ (0) += dt_ * d2com_ (0);
        dcom_ (1) += dt_ * d2com_ (1);

        // along z
        //d2com_ (2) = - (gainz_ (0)*z + gainz_ (1)*thetaz +
        //		gainz_ (2)*dcom_ (2) + gainz_ (3)*dthetaz);
        //dcom_ (2) += dt_ * d2com_ (2);
    }
    break;
    default:
        break;
    };

    comdot.resize (3);
    comdot [0].setSingleBound (comdotRef (0) + dcom_ (0));
    comdot [1].setSingleBound (comdotRef (1) + dcom_ (1));
    comdot [2].setSingleBound (comdotRef (2) + dcom_ (2));

    d2comSOUT_.setConstant (d2com_);
    d2comSOUT_.setTime (time);

    nbSupportSOUT_.setConstant (nbSupport_);
    nbSupportSOUT_.setTime (time);

    errorSOUT_.setConstant (dcom_);
    errorSOUT_.setTime (time);

    return comdot;
}

Matrix& HRP2DecoupledStabilizer::computeJacobianCom(Matrix& jacobian, const int& time)
{
    typedef unsigned int size_t;
    jacobian = jacobianSIN_ (time);
    return jacobian;
}

//    /// Compute flexibility state from both feet
//    void Stabilizer::computeFlexibility (const int& time)
//    {
//      const MatrixHomogeneous& flexibility = stateFlexSIN_.access (time);
//      const MatrixHomogeneous& Mr = rightFootPositionSIN_.access (time);
//      const MatrixHomogeneous& Ml = leftFootPositionSIN_.access (time);
//      const Vector& fr = forceRightFootSIN_.access (time);
//      const Vector& fl = forceLeftFootSIN_.access (time);
//      const Vector& forceRefLf = forceLeftFootRefSIN_.access (time);
//      const Vector& forceRefRf = forceRightFootRefSIN_.access (time);
//      const Vector& com = comSIN_ (time);
//      double deltaComRfx, deltaComRfy, deltaComLfx, deltaComLfy;
//      double dcomRfx, dcomRfy, dcomLfx, dcomLfy;
//
//      // compute component of angle orthogonal to the line joining the feet
//      double delta_x = Ml (0, 3) - Mr (0, 3);
//      double delta_y = Ml (1, 3) - Mr (1, 3);
//      double stepLength = sqrt (delta_x*delta_x+delta_y*delta_y);
//      stepLengthSOUT_.setConstant (stepLength);
//
//      u2x_ = delta_x/stepLength;
//      u2y_ = delta_y/stepLength;
//      u1x_ = u2y_;
//      u1y_ = -u2x_;
//
//      // Express vertical component of force in global basis
//      double flz = Ml (2,0) * fl (0) + Ml(2,1) * fl (1) + Ml (2,2) * fl (2);
//      double frz = Mr (2,0) * fr (0) + Mr(2,1) * fr (1) + Mr (2,2) * fr (2);
//
//      kth_ = flexRfx (4) + flexLfx (4);
//      nbSupport_ = 0;
//      if (on_) {
//	if (frz >= forceThreshold_) {
//	  nbSupport_++;
//	  supportCandidateRf_++;
//	  if (supportCandidateRf_ >= 3) {
//	    iterationsSinceLastSupportRf_ = 0;
//	  }
//	} else {
//	  supportCandidateRf_ = 0;
//	  iterationsSinceLastSupportRf_ ++;
//	}
//	if (flz >= forceThreshold_) {
//	  nbSupport_++;
//	  supportCandidateLf_++;
//	  if (supportCandidateLf_ >= 3) {
//	    iterationsSinceLastSupportLf_ = 0;
//	  }
//	} else {
//	  supportCandidateLf_ = 0;
//	  iterationsSinceLastSupportLf_++;
//	}
//      }
//      theta1RefPrev_ = theta1Ref_;
//      dtheta1Ref_ = 0.;
//      if (nbSupport_ == 2) {
//	// Compute reference moment from reference forces
//	double kthLat = .5*stepLength*stepLength*flexLat (4);
//	double Mu1Ref;
//	Mu1Ref = .5*(forceRefRf (2) - forceRefLf (2))*stepLength;
//	theta1Ref_ = Mu1Ref/kthLat;
//	dtheta1Ref_ = (theta1Ref_ - theta1RefPrev_)/dt_;
//      }
//
//      if (frz < 0) frz = 0;
//      if (flz < 0) flz = 0;
//      double Fz = flz + frz;
//      flexZobs_ (1) = Fz - m_ * g_;
//      flexLatObs_ (0) = u2x_* (com (0) - .5 * (Mr (0, 3) + Ml (0, 3))) +
//	u2y_ * ((com (1) - .5 * (Mr (1, 3) + Ml (1, 3))));
//      flexLatObs_ (1) = .5*stepLength*(frz - flz)
//	- (u1x_ * (Ml(0,0)*fl(3)+Ml(0,1)*fl(4)+Ml(0,2)*fl(5)) +
//	   u1y_ * (Ml(1,0)*fl(3)+Ml(1,1)*fl(4)+Ml(1,2)*fl(5)))
//	- (u1x_ * (Mr(0,0)*fr(3)+Mr(0,1)*fr(4)+Mr(0,2)*fr(5)) +
//	   u1y_ * (Mr(1,0)*fr(3)+Mr(1,1)*fr(4)+Mr(1,2)*fr(5)));
//      if (Fz == 0) {
//	flexValue_ (0) = 0;
//	flexValue_ (1) = 0;
//	flexDeriv_ (0) = 0;
//	flexDeriv_ (1) = 0;
//	deltaCom_ = comSIN_ (time) - comRefSIN_ (time);
//	return;
//      }
//      // Extract yaw from right foot position
//      double nx = Mr (0,0);
//      double ny = Mr (1,0);
//      double norm = sqrt (nx*nx + ny*ny);
//      double cth = nx/norm;
//      double sth = ny/norm;
//      // Flexibility right foot in global frame
//      double flexAngleRfx = cth * flexRfx (1) + sth * flexRfy (1);
//      double flexAngleRfy = -sth * flexRfx (1) + cth * flexRfy (1);
//      double flexDerivRfx = cth * flexRfx (3) + sth * flexRfy (3);
//      double flexDerivRfy = -sth * flexRfx (3) + cth * flexRfy (3);
//      // Compute deviation of center of mass
//      deltaComRfx = cth * flexRfx (0) - sth * flexRfy (0);
//      deltaComRfy = sth * flexRfx (0) + cth * flexRfy (0);
//      dcomRfx = cth * flexRfx (2) - sth * flexRfy (2);
//      dcomRfy = sth * flexRfx (2) + cth * flexRfy (2);
//      // Extract yaw from left foot position
//      nx = Ml (0,0);
//      ny = Ml (1,0);
//      norm = sqrt (nx*nx + ny*ny);
//      cth = nx/norm;
//      sth = ny/norm;
//      // Flexibility left foot in global frame
//      double flexAngleLfx = cth * flexLfx (1) + sth * flexLfy (1);
//      double flexAngleLfy = -sth * flexLfx (1) + cth * flexLfy (1);
//      double flexDerivLfx = cth * flexLfx (3) + sth * flexLfy (3);
//      double flexDerivLfy = -sth * flexLfx (3) + cth * flexLfy (3);
//
//      flexValue_ (0) = (frz * flexAngleRfx + flz * flexAngleLfx)/Fz;
//      flexValue_ (1) = (frz * flexAngleRfy + flz * flexAngleLfy)/Fz;
//      flexValue_ (2) = flexZ (1);
//      flexDeriv_ (0) = (frz * flexDerivRfx + flz * flexDerivLfx)/Fz;
//      flexDeriv_ (1) = (frz * flexDerivRfy + flz * flexDerivLfy)/Fz;
//      flexDeriv_ (2) = flexZ (3);
//      // Compute deviation of center of mass
//      deltaComLfx = cth * flexLfx (0) - sth * flexLfy (0);
//      deltaComLfy = sth * flexLfx (0) + cth * flexLfy (0);
//      dcomLfx = cth * flexLfx (2) - sth * flexLfy (2);
//      dcomLfy = sth * flexLfx (2) + cth * flexLfy (2);
//
//      deltaCom_ (0) = (frz * deltaComRfx + flz * deltaComLfx)/Fz;
//      deltaCom_ (1) = (frz * deltaComRfy + flz * deltaComLfy)/Fz;
//      deltaCom_ (2) = flexZ (0);
//      dcom_ (0) = (frz * dcomRfx + flz * dcomLfx)/Fz;
//      dcom_ (1) = (frz * dcomRfy + flz * dcomLfy)/Fz;
//      dcom_ (2) = flexZ (2);
//      // Compute flexibility transformations and velocities
//      if (Fz != 0) {
//	zmp_ (0) = (frz * Mr (0, 3) + flz * Ml (0, 3))/Fz;
//	zmp_ (1) = (frz * Mr (1, 3) + flz * Ml (1, 3))/Fz;
//	zmp_ (2) = (frz * Mr (2, 3) + flz * Ml (2, 3))/Fz;
//	uth_ (0) = flexValue_ (1);
//	uth_ (1) = -flexValue_ (0);
//	translation_ = zmp_ - R_ * zmp_;
//	uth_.toMatrix (R_);
//	for (std::size_t row = 0; row < 3; ++row) {
//	  for (std::size_t col = 0; col < 3; ++col) {
//	    flexPosition_ (row, col) = R_ (row, col);
//	  }
//	  flexPosition_ (row, 3) = translation_ (row);
//	}
//	// Lateral flexibility
//	double theta = flexLat (1);
//	uth_ (0) = u1x_ * theta;
//	uth_ (1) = u1y_ * theta;
//	uth_.toMatrix (R_);
//	translation_ = zmp_ - R_ * zmp_;
//	for (std::size_t row = 0; row < 3; ++row) {
//	  for (std::size_t col = 0; col < 3; ++col) {
//	    flexPositionLat_ (row, col) = R_ (row, col);
//	  }
//	  flexPositionLat_ (row, 3) = translation_ (row);
//	}
//
//	flexVelocity_ (0) = zmp_ (2) * flexDeriv_ (0);
//	flexVelocity_ (1) = zmp_ (2) * flexDeriv_ (1);
//	flexVelocity_ (2) = -zmp_ (0)*flexDeriv_ (0)-zmp_ (1)*flexDeriv_ (1);
//	flexVelocity_ (3) = flexDeriv_ (1);
//	flexVelocity_ (4) = -flexDeriv_ (0);
//
//	if (iterationsSinceLastSupportLf_ * dt_ >
//	    timeBeforeFlyingFootCorrection_) {
//	  flexPositionLf_ = flexPosition_;
//	  flexVelocityLf_ = flexVelocity_;
//	} else {
//	  flexPositionLf_.setIdentity ();
//	  flexVelocityLf_.setZero ();
//	}
//	if (iterationsSinceLastSupportRf_ * dt_ >
//	    timeBeforeFlyingFootCorrection_) {
//	  flexPositionRf_ = flexPosition_;
//	  flexVelocityRf_ = flexVelocity_;
//	} else {
//	  flexPositionRf_.setIdentity ();
//	  flexVelocityRf_.setZero ();
//	}
//	flexPositionSOUT_.setConstant (flexPosition_);
//	flexPositionLfSOUT_.setConstant (flexPositionLf_);
//	flexPositionRfSOUT_.setConstant (flexPositionRf_);
//	flexPositionLatSOUT_.setConstant (flexPositionLat_);
//	flexVelocitySOUT_.setConstant (flexVelocity_);
//	flexVelocityLfSOUT_.setConstant (flexVelocityLf_);
//	flexVelocityRfSOUT_.setConstant (flexVelocityRf_);
//	flexVelocityLatSOUT_.setConstant (flexVelocityLat_);
//      }
//    }



} // namespace dynamic
} // namespace sot

