###
import sys
import numpy as np
from dynamic_graph import plug
import dynamic_graph.signal_base as dgsb
from dynamic_graph.sot.core import Stack_of_vector, OpPointModifier, MatrixHomoToPose, Multiply_matrixHomo_vector
from dynamic_graph.sot.application.state_observation.initializations.hrp2_model_base_flex_estimator_imu_force_encoders import HRP2ModelBaseFlexEstimatorIMUForceEncoders
from dynamic_graph.sot.application.stabilizer.scenarii.seqplay_lqr_twoDof_coupled_stabilizer_hrp2_encoders import SeqPlayLqrTwoDofCoupledStabilizerHRP2Encoders
from dynamic_graph.sot.application.stabilizer import VectorPerturbationsGenerator
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.dynamics.zmp_from_forces import ZmpFromForces

def s() :
   stabilizer.stop()

forceSeqplay = True
#traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-left-foot'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-right-foot'
traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-resampled'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-resampled-30'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkfwd-shifted'
#traj = '/home/alexis/devel/ros/install/resources/seqplay/stand-on-left-foot-shifted' 
#traj = '/home/alexis/devel/ros/install/resources/seqplay/onspot16s'; forceSeqplay = False
#traj = '/home/alexis/devel/ros/install/resources/seqplay/onspot'; forceSeqplay = False
#traj = '/home/alexis/devel/ros/install/resources/seqplay/walkstraight10cmperstep'; forceSeqplay = False

appli =  SeqPlayLqrTwoDofCoupledStabilizerHRP2Encoders(robot, traj, False, False, False,forceSeqplay)
appli.withTraces()

estEnc = appli.taskCoMStabilized.estimatorEnc
stabilizer = appli.taskCoMStabilized
seq = appli.seq

# Changer raideurs
b="simu" # "simu"
if b=="robot" :
  # Simulation
  kfe=40000
  kfv=600
  kte=600
  ktv=60
elif b=="simu" :
  # Robot
  kfe=40000
  kfv=600
  kte=350
  ktv=10

stabilizer.setkts(kte)
stabilizer.setktd(ktv)
stabilizer.setkfs(kfe)
stabilizer.setkfd(kfv)

plug(robot.device.velocity,robot.dynamic.velocity)

zmp = ZmpFromForces('zmpReal')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

#zmpEnc = ZmpFromForces('zmpEncoders')
#plug (estEnc.forcesSupport1 , zmpEnc.force_0)
#plug (estEnc.forcesSupport2, zmpEnc.force_1)
#plug (estEnc.odometryFF.homoSupportPos1 , zmpEnc.sensorPosition_0)
#plug (estEnc.odometryFF.homoSupportPos2 , zmpEnc.sensorPosition_1)

appli.gains['trunk'].setConstant(2)
estEnc.setForceVariance(1e-4)
stabilizer.setFixedGains(False)
stabilizer.setHorizon(400)

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

appli.robot.addTrace( estEnc.odometry.name,'freeFlyer' )
appli.robot.addTrace( robot.dynamicEncoders.name,'position' )
appli.robot.addTrace( robot.dynamic.name,'position' )
appli.robot.addTrace( robot.dynamicOdo.name,'position' )

appli.robot.addTrace( robot.dynamic.name,'com' )
appli.robot.addTrace( robot.dynamicEncoders.name,'com' )
appli.robot.addTrace( robot.dynamic.name,'left-ankle' )
appli.robot.addTrace( robot.dynamicEncoders.name,'left-ankle' )

appli.robot.addTrace( estEnc.name,'flexibility' )
appli.robot.addTrace( estEnc.name,'state' )
appli.robot.addTrace( estEnc.name,'input')
appli.robot.addTrace( estEnc.interface.name,'contactsNbr')
appli.robot.addTrace( estEnc.name,'measurement')
appli.robot.addTrace( estEnc.name,  'forcesAndMoments')
appli.robot.addTrace( stabilizer.name,'task' )
appli.robot.addTrace ( stabilizer.name,'state')
appli.robot.addTrace ( stabilizer.name,'stateWorld')
appli.robot.addTrace ( stabilizer.name,'stateError')
appli.robot.addTrace ( stabilizer.name,'control')
appli.robot.addTrace (stabilizer.name,'gain')
appli.robot.addTrace (stabilizer.name,'Amatrix')
appli.robot.addTrace (stabilizer.name,'Bmatrix')
appli.robot.addTrace( zmp.name, 'zmp')
#appli.robot.addTrace( zmpEst.name, 'zmp')
#appli.robot.addTrace( zmpEnc.name, 'zmp')
appli.robot.addTrace (stabilizer.name,'computationTime')

appli.robot.addTrace( robot.dynamicEncoders.name,'position' )
appli.robot.addTrace( robot.dynamic.name,'position' )

appli.startTracer()


