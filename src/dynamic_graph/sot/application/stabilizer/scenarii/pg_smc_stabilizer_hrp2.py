from dynamic_graph.sot.application.stabilizer.scenarii.pg_lqr_twoDof_coupled_stabilizer import PgLqrTwoDofCoupledStabilizer
from dynamic_graph.sot.application.stabilizer.scenarii.hrp2_smc_stabilizer import HRP2SmcStabilizer
from dynamic_graph.sot.core.meta_tasks import GainAdaptive
from dynamic_graph import plug
from dynamic_graph.sot.core.matrix_util import matrixToTuple
from dynamic_graph.sot.core import  MatrixToUTheta, HomoToMatrix, HomoToRotation, Multiply_matrix_vector
from numpy import diag


class PgSMCStabilizerHRP2(PgLqrTwoDofCoupledStabilizer):
    def __init__(self,robot,trunkStabilize = False, hands = False, posture =False):
        PgLqrTwoDofCoupledStabilizer.__init__(self,robot,trunkStabilize,hands,posture)

    def createStabilizedCoMTask (self):
        task = HRP2SmcStabilizer(self.robot)
        gain = GainAdaptive('gain'+task.name)
        plug(self.comRef,task.comRef)
	task.waistOriRef.value=(0,)*3
        task.flexOriRef.value=(0,)*3
        task.comDotRef.value=(0,)*3
        task.waistVelRef.value=(0,)*3
        task.flexAngVelRef.value=(0,)*3
        plug(gain.gain, task.controlGain)
        plug(task.error, gain.error) 
        return (task, gain)
    
    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 10
        weight_head      = 0.3
        weight_arm       = 1

        #weight = diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        #weight[9,9] = weight_knee
        #weight[15,15] = weight_knee
        #weight[19,19] = weight_chesttilt

        weight = diag( (0,)*6+(1,)*30)
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        #mask = '1'*36
        #mask = '1'*14+'0'*22
        #self.tasks['posture'].controlSelec.value = mask
