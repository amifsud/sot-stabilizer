#include <sot-stabilizer/tools/hrp2.hpp>
#include <sot-state-observation/tools/definitions.hh>
#include <sot-stabilizer/hrp2-smc-stabilizer.hh>
#include <state-observation/tools/miscellaneous-algorithms.hpp>

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

    // Input initialization

    /// Definitions of input vectors

    /// Definition of ouptut vectors
    IndexedMatrixArray x;
    IndexedMatrixArray u;

    HRP2SMCStabilizer stabilizer("Stabilizer");

    return 0;
}

int main()
{

    return testModel();

}
