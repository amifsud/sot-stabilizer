 ###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
from dynamic_graph.sot.application.stabilizer.scenarii.pg_lqr_twoDof_coupled_stabilizer_hrp2 import PgLqrTwoDofCoupledStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

appli =  PgLqrTwoDofCoupledStabilizerHRP2(robot, False, False, False)
appli.withTraces()

est = appli.taskCoMStabilized.estimator
stabilizer = appli.taskCoMStabilized
seq = appli.seq

# Changer raideurs

# Simulation
#kfe=40000
#kfv=600
#kte=600
#ktv=60

# Robot
stabilizer.setkts(350)
stabilizer.setktd(10)
stabilizer.setkfs(5000)
stabilizer.setkfd(600)

plug(robot.device.velocity,robot.dynamic.velocity)

zmp = ZmpFromForces('zmpReal')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

zmpInterface = ZmpFromForces('zmpInterface')
plug (est.interface.forceSupport1 , zmpInterface.force_0)
plug (est.interface.forceSupport2 , zmpInterface.force_1)
plug (est.interface.positionSupport1, zmpInterface.sensorPosition_0)
plug (est.interface.positionSupport2, zmpInterface.sensorPosition_1)

zmpEst = ZmpFromForces('zmpEstimated')
plug (est.forcesSupport1 , zmpEst.force_0)
plug (est.forcesSupport2, zmpEst.force_1)
plug (est.interface.positionSupport1 , zmpEst.sensorPosition_0)
plug (est.interface.positionSupport2, zmpEst.sensorPosition_1)

est.interface.setWithUnmodeledMeasurements(False)
est.interface.setWithModeledForces(True)
est.interface.setWithAbsolutePose(False)
est.setWithComBias(False)

# Covariances
est.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*12+(1.e-2,)*6+(1e-8,)*2+(1.e-8,)*3)))
est.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3))) 
est.setUnmodeledForceVariance(1e-13)
est.setForceVariance(1e-8)
est.setAbsolutePosVariance(1e-4)

# Contact model definition
est.setKfe(matrixToTuple(np.diag((40000,40000,40000))))
est.setKfv(matrixToTuple(np.diag((600,600,600))))
est.setKte(matrixToTuple(np.diag((600,600,600))))
est.setKtv(matrixToTuple(np.diag((60,60,60))))

#est.initAbsolutePoses()

appli.gains['trunk'].setConstant(2)
stabilizer.setFixedGains(True)
stabilizer.setHorizon(400)
est.setOn(True)

appli.robot.addTrace( est.name,'state' )
appli.robot.addTrace( est.name,'momentaFromForces')
appli.robot.addTrace( est.name,'momentaFromKinematics')
appli.robot.addTrace( est.name,'contactNbr' )
appli.robot.addTrace( est.interface.name,'inputConstSize')
appli.robot.addTrace( est.interface.name,'measurementConstSize')
appli.robot.addTrace( zmp.name, 'zmp')
appli.robot.addTrace( zmpInterface.name, 'zmp')
appli.robot.addTrace( zmpEst.name, 'zmp')
#appli.robot.addTrace( robot.device.name, 'control')
#appli.robot.addTrace( robot.device.name, 'state')

appli.startTracer()
appli.nextStep()
appli.nextStep(3)
appli.nextStep(2)

# Perturbation Generator on control
perturbatorControl = VectorPerturbationsGenerator('perturbatedControl')
perturbatorControl.setSinLessMode(True)
vect1 = perturbatorControl.sin
vect1.value = (0,0,0,0,0)
plug (perturbatorControl.sout,stabilizer.perturbationAcc)
appli.robot.addTrace( perturbatorControl.name, 'sout')
perturbatorControl.perturbation.value=(1,1,0,1,1)
perturbatorControl.selec.value = '11111'
perturbatorControl.setMode(2)
perturbatorControl.activate(False)

# Perturbation Generator on task
perturbatorTask = VectorPerturbationsGenerator('perturbatedTask')
perturbatorTask.setSinLessMode(True)
vect = perturbatorTask.sin
vect.value = (0,0,0,0,0)
plug (perturbatorTask.sout,stabilizer.perturbationVel)
appli.robot.addTrace( perturbatorTask.name, 'sout')
perturbatorTask.perturbation.value=(1,0,0,0,0)
perturbatorTask.selec.value = '11111'
perturbatorTask.setMode(0)
perturbatorTask.setPeriod(0)
perturbatorTask.activate(False)


