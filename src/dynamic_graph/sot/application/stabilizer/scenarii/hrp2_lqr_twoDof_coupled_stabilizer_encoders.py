# Launch it with py ../robotViewerLauncher.py +compensater.py +appli.py 

from dynamic_graph.sot.application.stabilizer import HRP2LQRTwoDofCoupledStabilizer
from dynamic_graph import plug
import numpy as np
import dynamic_graph.signal_base as dgsb
    
from dynamic_graph.sot.core import Stack_of_vector, MatrixHomoToPoseUTheta, OpPointModifier, Multiply_matrix_vector, MatrixHomoToPose, MatrixHomoToPoseRollPitchYaw, MatrixToUTheta, HomoToMatrix, HomoToRotation
from dynamic_graph.sot.core.matrix_util import matrixToTuple

class HRP2LqrTwoDofCoupledStabilizerEncoders(HRP2LQRTwoDofCoupledStabilizer):
    
    def __init__(self,robot,taskname = 'com-stabilized'):      

	from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force_encoders import HRP2ModelBaseFlexEstimatorIMUForceEncoders
        HRP2LQRTwoDofCoupledStabilizer.__init__(self,taskname)

	robot.device.state.value=robot.halfSitting
	robot.device.robotState.value=robot.halfSitting
	robot.device.velocity.value=(0.,)*36
	robot.device.forceLLEG.value=(1.8349814919184242, -7.4412430930486302, 256.06853454222494, -0.035428813912447302, 2.0798475785647286, 0.14169701504511384)
	robot.device.forceRLEG.value=(2.4303733340459406, 11.156361786170869, 285.59013529212666, -0.69957871247984049, 2.0516111892090887, -0.22872430884228223)
	robot.device.accelerometer.value=(0.12527866711822, 0.089756740219665537, 9.8059788372472152)
	robot.device.gyrometer.value=(-0.0029326257213877862, 0.007655425240526083, -8.001571126249214e-05)

	def recomputeDynamic(i,dynamic):
		dynamic.chest.recompute(i)
		dynamic.com.recompute(i)
		dynamic.Jcom.recompute(i)
		dynamic.angularmomentum.recompute(i)
		dynamic.inertia.recompute(i)
		dynamic.waist.recompute(i)
		dynamic.signal('left-ankle').recompute(i)
		dynamic.signal('right-ankle').recompute(i)
	recomputeDynamic(0,robot.dynamic)

	# Reconstruction of the control state

		# DCoM
        self.DCom = Multiply_matrix_vector('DCom') 
        plug(robot.dynamic.Jcom,self.DCom.sin1)
        plug(robot.device.velocity,self.DCom.sin2)

		# DWaist
        self.DWaist = Multiply_matrix_vector('DWaist') 
        plug(robot.dynamic.Jwaist,self.DWaist.sin1)
        plug(robot.device.velocity,self.DWaist.sin2)

		# Estimator of the flexibility state
        self.estimatorEnc = HRP2ModelBaseFlexEstimatorIMUForceEncoders (robot, taskname+"EstimatorEncoders")
	plug (self.estimatorEnc.odometryFF.nbSupport,self.nbSupport)
	plug(self.estimatorEnc.flexPosition, self.tflex)
	plug(self.estimatorEnc.flexVelocity, self.dtflex)
	plug(self.estimatorEnc.flexAcceleration, self.ddtflex)

	recomputeDynamic(2,robot.dynamic)
	recomputeDynamic(2,robot.dynamicEncoders)
	recomputeDynamic(2,robot.dynamicFF)
	self.estimatorEnc.state.recompute(2)

	# Control state
	plug(robot.dynamic.com, self.com)
	plug(robot.dynamic.waist,self.waistHomo)
        plug(self.estimatorEnc.flexThetaU, self.flexOriVect ) 
	plug(self.DCom.sout,self.comDot)
	plug(self.DWaist.sout,self.waistAngVel)
        plug(self.estimatorEnc.flexOmega, self.flexAngVelVect )

	# Jacobians
        plug (robot.dynamic.Jcom, self.Jcom)
	plug (robot.dynamic.Jchest, self.Jchest)
	plug ( robot.dynamic.Jwaist, self.Jwaist) 

	# Inertia
	plug ( robot.dynamic.inertia, self.inertia)
        plug(robot.dynamic.angularmomentum,self.angularmomentum)



