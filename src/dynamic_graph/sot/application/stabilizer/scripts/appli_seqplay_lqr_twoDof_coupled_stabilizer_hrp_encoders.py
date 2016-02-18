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
plug(robot.device.velocity,robot.dynamicFF.velocity)
plug(robot.device.velocity,robot.dynamicEncoders.velocity)

# Simulation
appli.comRef.value=(0.01068, 0.00014, 0.80771000000000004) # two feet Mx=My=0
#appli.comRef.value=(0.00182, 0.09582, 0.80769999999999997) # one foot Mx=My=0

# Robot
appli.comRef.value=(0.0263, 0.0040000000000000001, 0.80771000000000004) # zmp au niveau du pied
estEnc.inputVector.setFootBias1((0.0168,0,0)) # zmpEst=zmp sans feedback
estEnc.inputVector.setFootBias2((0.0168,0,0))
estEnc.inputVector.setFootBias1((0.0175,0,0,0,0,0)) # zmpEst=zmp avec feedback
estEnc.inputVector.setFootBias2((0.0175,0,0,0,0,0))
estEnc.inputVector.setFootBias1((0,0,0,0,0,0)) # sans biai
estEnc.inputVector.setFootBias2((0,0,0,0,0,0))

zmp = ZmpFromForces('zmpReal')
plug (robot.device.forceLLEG , zmp.force_0)
plug (robot.device.forceRLEG, zmp.force_1)
plug (robot.frames['leftFootForceSensor'].position , zmp.sensorPosition_0)
plug (robot.frames['rightFootForceSensor'].position, zmp.sensorPosition_1)

zmpEnc = ZmpFromForces('zmpEncoders')
plug (estEnc.forcesSupport1 , zmpEnc.force_0)
plug (estEnc.forcesSupport2, zmpEnc.force_1)
plug (estEnc.odometryFF.homoSupportPos1 , zmpEnc.sensorPosition_0)
plug (estEnc.odometryFF.homoSupportPos2 , zmpEnc.sensorPosition_1)

appli.gains['trunk'].setConstant(2)
estEnc.setMeasurementNoiseCovariance(matrixToTuple(np.diag((1e-3,)*3+(1e-6,)*3)))
#estEnc.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6)))
estEnc.setForceVariance(1e-4)

stabilizer.setFixedGains(False)
stabilizer.setHorizon(400)
estEnc.inputVector.setConfig((1,1,1))
#estEnc.setWithForceSensors(True)
#estEnc.setWithComBias(False)
#estEnc.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6)))
##estEnc.setProcessNoiseCovariance(matrixToTuple(np.diag((1e-8,)*12+(1e-4,)*6+(2.5e-8,)*2)))

stabilizer.setStateCost(matrixToTuple(1*np.diag((100,100,1000,100,100,100,100,1,1,100,1,1,1,1))))

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

#appli.nextStep()

appli.robot.addTrace( estEnc.name,'flexibility' )
appli.robot.addTrace( estEnc.name,'input')
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
appli.robot.addTrace( zmpEst.name, 'zmp')
appli.robot.addTrace( zmpEnc.name, 'zmp')
appli.robot.addTrace (stabilizer.name,'computationTime')

appli.robot.addTrace( robot.dynamicEncoders.name,'position' )
appli.robot.addTrace( robot.dynamic.name,'position' )
appli.robot.addTrace( robot.dynamicFF.name,'position' )
appli.robot.addTrace( estEnc.odometryFF.name,'nbSupport' )

appli.robot.addTrace( estEnc.odometryFF.name,'supportPos1' )
appli.robot.addTrace( estEnc.odometryFF.name,'supportPos2' )

appli.robot.addTrace( estEnc.odometryFF.name,'robotStateOut' )

appli.robot.addTrace( estEnc.odometryFF.name,'pivotPosition' )

appli.robot.addTrace( estEnc.odometryFF.name,'forceSupport1' )
appli.robot.addTrace( estEnc.odometryFF.name,'forceSupport2' )

appli.robot.addTrace( estEnc.odometryFF.name,'rightFootPosition')
appli.robot.addTrace( estEnc.odometryFF.name,'leftFootPosition')

from dynamic_graph.sot.application.state_observation import Filter
filtre=Filter("Filter")
filtre.setWindowSize(10)
plug(zmp.zmp,filtre.sin)
appli.robot.addTrace( filtre.name,'sout' )

appli.startTracer()

stabilizer.setStateCost(matrixToTuple(1*np.diag((200000,2000,100,400,6000,20,1000,10,10,1000,1,10,4,1))))

# Sur deux pieds

# without force sensor
stabilizer.setStateCost(matrixToTuple(1*np.diag((32000,1,10000,1,200,1,24000,10,1,1000,1,10,1,120)))) 

# with force sensor
stabilizer.setStateCost(matrixToTuple(1*np.diag((100000,2000,10000,400,6000,2000,48000,10,10,1000,1,10,400,120))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((200000,2000,10000,400,6000,2000,100000,10,10,1000,1,10,400,120)))) # plus reactif com en x
stabilizer.setStateCost(matrixToTuple(1*np.diag((200000,200,10000,200,6000,200,100000,10,1,1000,0.1,1,400,120))))


# Sur un pied

# with force sensor
stabilizer.setStateCost(matrixToTuple(1*np.diag((200000,200000,10000,6000,6000,100,100,10,10,1000,10,10,1,1))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((200000,200000,10000,6000,6000,200000,200000,10,10,1000,10,10,240,240)))) # plus reactif
stabilizer.setStateCost(matrixToTuple(1*np.diag((2000,2000,10000,6000,6000,100,100,1,1,1000,10,10,1,1))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((2000,2000,1000,60000,60000,100,100,0.1,0.1,10,100,100,1,1))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((2000,20,1000,6000000000,6000000000,32000,2000,0.1,0.1,10,10000000,10000000,40,20))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((2000,200,1000,6000000000,6000000000,14000,14000,0.1,0.1,10,10000000,10000000,160,3200))))






stabilizer.setStateCost(matrixToTuple(1*np.diag((1000,1000,10000,1000,1000,1000,1000,10,10,1000,10,10,10,10))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((300,300,3000,300,300,300,300,3,3,300,3,3,3,3))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((100,100,1000,100,100,100,100,1,1,100,1,1,1,1))))
stabilizer.setStateCost(matrixToTuple(1*np.diag((32000,32000,10000,200,200,24000,24000,10,10,1000,10,10,120,120))))

stabilizer.setStateCost(matrixToTuple(1*np.diag((32000,32000,10000,20000,20000,24000,24000,10,10,1000,1000,1000,120,120))))






