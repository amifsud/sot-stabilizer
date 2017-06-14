#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/controllers/discrete-time-ordinary-smc.hh>
#include <sot-stabilizer/prototyping/non-linear-rotational-table-cart-device.hh>

#include <iostream>

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
    const unsigned kcontrol = 1500;
    const unsigned kmax=5000;

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
    x0.segment(0,3) << 0.05,
                       0,
                       0.8;
    system.setState(convertVector<dynamicgraph::Vector>(x0));

    // Stabilizer initialization
    controller::DiscreteTimeOrdinarySMC stabilizer(stateSize, controlSize);
    stateObservation::Vector x; x.resize(stateSize);
    stateObservation::Vector u; u.resize(controlSize); u.setZero();
    IndexedMatrixArray state, control;
    stabilizer.setStateDerivativeRef(stateObservation::Vector::Zero(stateSize)); // stabilization around an equilibrium
    stateObservation::Matrix lambdaa, alpha0, lambda0;
    lambdaa.resize(controlSize,controlSize);
    alpha0.resize(controlSize,stabilizer.getUnderActuatedSize());
    lambda0.resize(controlSize,stabilizer.getUnderActuatedSize());
    lambdaa << 10,0,0,0,0,0,
               0,10,0,0,0,0,
               0,0,10,0,0,0,
               0,0,0,10,0,0,
               0,0,0,0,10,0,
               0,0,0,0,0,10;
    lambda0 << 10,0,0,
               0,10,0,
               0,0,10,
               0,10,0,
               -10,0,0,
               0,0,10;
    alpha0 <<  1,0,0,
               0,1,0,
               0,0,1,
               0,1,0,
               -1,0,0,
               0,0,1;
    stabilizer.setLambdaa(lambdaa);
    stabilizer.setLambda0(lambda0);
    stabilizer.setAlpha0(alpha0);

    // State reference
    stateObservation::Vector xRef; xRef.resize(stateSize);
    xRef.setZero();
    xRef[0]=0.05;
    xRef[2]=0.8;

    for(unsigned k=kinit; k<kmax; k++)
    {
        // Integrate system
        system.computeDynamics(dt,u);
        x = system.getState();

        // Compute next control
        stabilizer.setState(x.segment(0,stateSize),k);
        stabilizer.setStateDerivative(x.segment(stateSize+3,stateSize));
        stabilizer.setStateRef(xRef);
        if(k>kcontrol) u = stabilizer.getControl(k-kcontrol-1);

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
