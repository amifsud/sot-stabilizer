###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force import HRP2ModelBaseFlexEstimatorIMUForce
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_lqr_twoDof_coupled_stabilizer_hrp2 import SeqPlayLqrTwoDofCoupledStabilizerHRP2
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

forceSeqplay = True
#traj = '/home/amifsud/devel/ros/install/resources/seqplay/stand-on-left-foot'
traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-right-foot'
traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-resampled'

appli =  SeqPlayLqrTwoDofCoupledStabilizerHRP2(robot, traj, False, False, False,forceSeqplay)
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

identity = ((1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1))

zmpEst = ZmpFromForces('zmpEstimated')
plug (est.forcesSupport1 , zmpEst.force_0)
plug (est.forcesSupport2, zmpEst.force_1)
zmpEst.sensorPosition_0.value = identity
zmpEst.sensorPosition_1.value = identity

est.interface.setWithUnmodeledMeasurements(False)
est.interface.setWithModeledForces(True)
est.interface.setWithAbsolutePose(False)
est.setWithComBias(False)

# Covariances
est.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*12+(1.e-2,)*6+(1e-15,)*2+(1.e-8,)*3)))
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

appli.robot.addTrace( est.name,'flexibility' )
appli.robot.addTrace( est.name,'state' )
appli.robot.addTrace( est.name,'inovation' )
appli.robot.addTrace( est.interface.name,'inputConstSize')
appli.robot.addTrace( est.interface.name,'measurementConstSize')
appli.robot.addTrace( est.interface.name,'input')
appli.robot.addTrace( est.interface.name,'measurement')
appli.robot.addTrace( est.name,  'forcesAndMoments')
appli.robot.addTrace( est.name,  'forcesSupport1')
appli.robot.addTrace( est.name,  'forcesSupport2')
appli.robot.addTrace( stabilizer.name,'nbSupport' )
appli.robot.addTrace( stabilizer.name,'error' )
appli.robot.addTrace( robot.device.name, 'forceLLEG')
appli.robot.addTrace( robot.device.name, 'forceRLEG')
appli.robot.addTrace( robot.device.name, 'forceLARM')
appli.robot.addTrace( robot.device.name, 'forceRARM')
appli.robot.addTrace( robot.device.name, 'accelerometer')
appli.robot.addTrace( robot.device.name, 'gyrometer')
appli.robot.addTrace ( stabilizer.name,'state')
appli.robot.addTrace ( stabilizer.name,'stateWorld')
appli.robot.addTrace ( stabilizer.name,'stateRef')
appli.robot.addTrace ( stabilizer.name,'stateError')
appli.robot.addTrace ( stabilizer.name,'stateExtended')
appli.robot.addTrace ( stabilizer.name,'control')
appli.robot.addTrace (robot.device.name,'velocity')
appli.robot.addTrace (robot.dynamic.name,'angularmomentum')
appli.robot.addTrace (robot.dynamic.name,'waist')
appli.robot.addTrace (stabilizer.name,'inertiaOut')
appli.robot.addTrace (stabilizer.name,'gain')
appli.robot.addTrace (stabilizer.name,'supportPos1')
appli.robot.addTrace (stabilizer.name,'supportPos2')
appli.robot.addTrace( zmp.name, 'zmp')
appli.robot.addTrace( zmpEst.name, 'zmp')
appli.robot.addTrace( est.interface.name, 'modeledContactsNbr')
appli.robot.addTrace( est.interface.name, 'contactsNbr')

appli.startTracer()
appli.nextStep()
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


