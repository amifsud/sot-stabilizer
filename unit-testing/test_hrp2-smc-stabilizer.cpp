#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

#include <sot-stabilizer/hrp2-smc-stabilizer.hh>
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

    const unsigned stateSize=6;
    const unsigned controlSize=3;

    /// System initialization
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
    contactPos1 << 1,0,0,0.095,
                   0,1,0,0.15,
                   0,0,1,0,
                   0,0,0,1;
    contactPos2 << 1,0,0,0.095,
                   0,1,0,-0.15,
                   0,0,1,0,
                   0,0,0,1;
    system.setContactPosition(0,contactPos1);
    system.setContactPosition(1,contactPos2);
    dynamicgraph::Vector x0; x0.resize(stateSize); x0.setZero();
    system.setState(x0);

    /// Stabilizer initialization
    HRP2SMCStabilizer stabilizer("Stabilizer");


    IndexedMatrixArray u;
    IndexedMatrixArray x;



    return 0;
}

int main()
{

    return testModel();

}
