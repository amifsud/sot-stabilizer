#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/controllers/discrete-time-ordinary-smc.hh>
#include <sot-stabilizer/prototyping/non-linear-rotational-table-cart-device.hh>

using namespace sotStabilizer;
using namespace sotStateObservation;
using namespace stateObservation;

int testModel()
{
    /// sampling period
    const double dt=5e-3;

    /// Initializations

    // Dimensions
    const unsigned kinit=0;
    const unsigned kmax=1400;

    const unsigned stateSize=9;
    const unsigned controlSize=6;

    // System initialization
    NonLinearRotationalTableCartDevice system("System");
    system.setRobotMass(56.8);
    system.setContactsNumber(2);
    system.setKfe(convertMatrix<dynamicgraph::Matrix>(10000*Matrix3::Identity()));
    system.setKfv(convertMatrix<dynamicgraph::Matrix>(600*Matrix3::Identity()));
    system.setKte(convertMatrix<dynamicgraph::Matrix>(600*Matrix3::Identity()));
    system.setKtv(convertMatrix<dynamicgraph::Matrix>(60*Matrix3::Identity()));
    stateObservation::Matrix3 I;
    I << 48.0,0,0,
         0,47.0,0,
         0,0,2;
    system.setMomentOfInertia(convertMatrix<dynamicgraph::Matrix>(I));
    stateObservation::Matrix4 contactPos1, contactPos2;
    contactPos1 << 1,0,0,0,
                   0,1,0,0.15,
                   0,0,1,0,
                   0,0,0,1;
    contactPos2 << 1,0,0,0,
                   0,1,0,-0.15,
                   0,0,1,0,
                   0,0,0,1;
    system.setContactPosition(0,contactPos1);
    system.setContactPosition(1,contactPos2);
    stateObservation::Vector x0; x0.resize(2*stateSize+6); x0.setZero();
    x0.segment(0,3) << 0,
                       0,
                       0.8;
    system.setState(convertVector<dynamicgraph::Vector>(x0));

    // Stabilizer initialization
    controller::DiscreteTimeOrdinarySMC stabilizer(stateSize, controlSize);
    stateObservation::Vector x; x.resize(stateSize);
    stateObservation::Vector dx; dx.resize(stateSize);
    stateObservation::Vector u; u.resize(stateSize); u.setZero();
    IndexedMatrixArray state, control;

    // State reference
    stateObservation::Vector xRef; xRef.resize(stateSize);
    xRef.setZero();

    stabilizer.setStateDerivativeRef(stateObservation::Vector::Zero(stateSize));

    for(unsigned k=kinit; k<kmax; k++)
    {
        // Integrate system
        system.computeDynamics(dt,u);
        x = system.getState();

        // Compute next control
        stabilizer.setState(x.segment(0,stateSize));
        stabilizer.setStateDerivative(x.segment(stateSize+3,stateSize));
        stabilizer.setStateRef(xRef);
        u = stabilizer.getControl(k);

        // Save state
        state.setValue(x,k);
        control.setValue(u,k);
    }

    state.writeInFile("state.dat");
    control.writeInFile("control.dat");

    return 0;
}

int main()
{
    return testModel();
}
