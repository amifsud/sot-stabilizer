# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2SMCStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw, MatrixToUTheta, HomoToMatrix, HomoToRotation
from dynamic_graph.sot.core.matrix_util import matrixToTuple



class HRP2SmcStabilizer(HRP2SMCStabilizer):
    
    
    def __init__(self,robot,taskname = 'com-stabilized'):      

	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce  
        HRP2SMCStabilizer.__init__(self,taskname)

	robot.device.state.value=robot.halfSitting
	robot.device.velocity.value=(0.,)*36
	robot.device.forceLLEG.value=(1.8349814919184242, -7.4412430930486302, 256.06853454222494, -0.035428813912447302, 2.0798475785647286, 0.14169701504511384)
	robot.device.forceRLEG.value=(2.4303733340459406, 11.156361786170869, 285.59013529212666, -0.69957871247984049, 2.0516111892090887, -0.22872430884228223)
	robot.device.accelerometer.value=(0.12527866711822, 0.089756740219665537, 9.8059788372472152)
	robot.device.gyrometer.value=(-0.0029326257213877862, 0.007655425240526083, -8.001571126249214e-05)
	robot.device.forceLARM.value=(-2.04071, 3.25524, -5.89116, 0.0814646, 0.0779619, 0.0190317)
	robot.device.forceRARM.value=(2.07224, -9.57906, 0.69111, -0.415802, 0.289026, 0.00621748)

	def recomputeDynamic(i):
		robot.dynamic.chest.recompute(i)
		robot.dynamic.com.recompute(i)
		robot.dynamic.Jcom.recompute(i)
		robot.dynamic.angularmomentum.recompute(i)
		robot.dynamic.inertia.recompute(i)
		robot.dynamic.waist.recompute(i)
	recomputeDynamic(0)

	# DCoM
        self.DCom = Multiply_matrix_vector('DCom') # Com velocity: self.DCom.sout
        plug(robot.dynamic.Jcom,self.DCom.sin1)
        plug(robot.device.velocity,self.DCom.sin2)

	# DWaist
        self.DWaist = Multiply_matrix_vector('DWaist') # Waist angulat velocity: self.DWaist.sout
        plug(robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(robot.device.velocity,self.DWaist.sin2)

	# Estimator of the flexibility state
        self.estimator = HRP2ModelBaseFlexEstimatorIMUForce (robot, taskname+"Estimator")
        plug (self.estimator.interface.modeledContactsNbr,self.nbSupport)

	self.estimator.state.recompute(2)
	recomputeDynamic(2)

	# Control state
	plug(robot.dynamic.com, self.com)
	plug(robot.dynamic.waist,self.waistHomo)
        plug(self.estimator.flexThetaU, self.flexOriVect )
	plug(self.DCom.sout,self.comDot)
	plug(self.DWaist.sout,self.waistAngVel)
        plug(self.estimator.flexOmega, self.flexAngVelVect )

	# CoM bias
	plug(self.estimator.comBias,self.comBias)

	# Jacobians
        plug ( robot.dynamic.Jcom  , self.Jcom  )
	plug ( robot.dynamic.Jwaist, self.Jwaist) 
	plug ( robot.dynamic.Jchest, self.Jchest)



