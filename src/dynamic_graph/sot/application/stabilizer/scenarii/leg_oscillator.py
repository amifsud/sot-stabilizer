from dynamic_graph.sot.core.meta_task_6d import MetaTask6d,toFlags
from dynamic_graph.sot.core.meta_task_visual_point import MetaTaskVisualPoint
from dynamic_graph.sot.core.meta_tasks import *
import  numpy as np
from numpy.linalg import inv,pinv,norm
from dynamic_graph.sot.core.matrix_util import matrixToTuple, vectorToTuple,rotate,matrixToRPY
import dynamic_graph.sot.core.matrix_util 
rpyToMatrix = RPYToMatrix
#from dynamic_graph.sot.core.meta_tasks_relative import gotoNdRel, MetaTask6dRel
from dynamic_graph.sot.core.meta_task_posture import MetaTaskPosture
from dynamic_graph.sot.core.feature_vector3 import FeatureVector3
from dynamic_graph.tracer_real_time import *
from dynamic_graph.sot.tools import Oscillator
from dynamic_graph.sot.core import Stack_of_vector, PoseRollPitchYawToMatrixHomo, Multiply_double_vector

from dynamic_graph.sot.application.state_observation import MovingFrameTransformation
from dynamic_graph.sot.application.velocity.precomputed_tasks import Application

toList = lambda sot: map(lambda x: x[2:],sot.display().split('\n')[3:-2])

def change6dPositionReference(task,feature,gain,position,selec=None,ingain=None,resetJacobian=True):
    M=generic6dReference(position)
    if selec!=None:
        if isinstance(selec,str):  feature.selec.value = selec
        else: feature.selec.value = toFlags(selec)
    feature.reference.value = matrixToTuple(M)
    if gain!=None:  setGain(gain,ingain)
    if 'resetJacobianDerivative' in task.__class__.__dict__.keys() and resetJacobian:
        task.resetJacobianDerivative()
        
def createTrunkTask (robot, application, taskName, ingain = 1.):
    task = Task (taskName)
    task.add (application.features['chest'].name)
    task.add (application.features['waist'].name)
    task.add (application.features['gaze'].name)
    gain = GainAdaptive('gain'+taskName)
    gain.setConstant(ingain)
    plug(gain.gain, task.controlGain)
    plug(task.error, gain.error)      
    return (task, gain)

class LegOscillator(Application):

    threePhaseScrew = True
    tracesRealTime = True

    def __init__(self,robot,twoHands = True,trunkStabilize = True):
        Application.__init__(self,robot)

        self.twoHands = twoHands
        self.trunkStabilize = trunkStabilize

        self.robot = robot
        plug(self.robot.dynamic.com,self.robot.device.zmp)
        
        self.sot = self.solver.sot

        self.createTasks()
        self.initTasks()
        self.initTaskGains()
        self.initOscillator()

        self.initialStack()

    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    # --- TASKS --------------------------------------------------------------------
    def createTasks(self):
        (self.tasks['trunk'],self.gains['trunk'])= createTrunkTask (self.robot, self, 'Tasktrunk')
        self.taskbalance = self.tasks['balance']
        self.taskRH      = self.tasks['right-wrist']
        self.taskLH      = self.tasks['left-wrist']
        self.taskPosture = self.tasks['posture']
        self.taskTrunk   = self.tasks['trunk']
        self.taskHalfStitting = MetaTaskPosture(self.robot.dynamic,'halfsitting')
	
        

    #initialization is separated from the creation of the tasks because if we want to switch
    #to second order controlm the initialization will remain while the creation is 
    #changed

    def initTasks(self):
        self.initTaskTrunk()
        self.initTaskPosture()
        #self.initTaskHalfSitting()
        self.initTaskCompensate()

    #def initTaskHalfSitting(self):
    #    self.taskHalfStitting.gotoq(None,self.robot.halfSitting)

    def initTaskTrunk(self):
        # --- BALANCE ---
        self.features['chest'].frame('desired')
        self.features['waist'].frame('desired')
        self.features['gaze'].frame('desired')
        #self.taskChest.feature.selec.value = '111111'
        self.features['chest'].selec.value = '111000'
        self.features['waist'].selec.value = '111111'
        self.features['gaze'].selec.value = '111000'
        self.featureCom.selec.value = '011'

    def setOsciFreq(self, f):
        self.oscillatorPitch.omega.value = f * 3.1415
        self.oscillatorYaw.omega.value = f * 3.1415
        
    def setOsciMagni(self,m):
        self.oscillatorPitch.magnitude.value = m
        self.oscillatorYaw.magnitude.value = m

    def initOscillator(self):
        self.oscillatorPitch = Oscillator('oscillatorPitch')
        self.oscillatorPitch.setContinuous(True)
        self.oscillatorPitch.setActivated(True)
        self.oscillatorPitch.setTimePeriod(self.robot.timeStep)
        self.oscillatorPitch.setActivated(False)
        self.oscillatorPitch.magnitude.value = 0.1
        self.oscillatorPitch.phase.value = 0.0
        self.oscillatorPitch.omega.value = 0.75
        
        self.oscillatorYaw = Oscillator('oscillatorYaw')
        self.oscillatorYaw.setContinuous(True)
        self.oscillatorYaw.setActivated(True)
        self.oscillatorYaw.setTimePeriod(self.robot.timeStep)
        self.oscillatorYaw.setActivated(False)
        self.oscillatorYaw.magnitude.value = 0.1
        self.oscillatorYaw.phase.value = 1.57
        self.oscillatorYaw.omega.value = 0.75

        self.minuspitch =  Multiply_double_vector('minuspitch')
        self.minuspitch.sin1.value = -1
        plug (self.oscillatorPitch.vectorSout, self.minuspitch.sin2)

        self.chestyaw =  Multiply_double_vector('chestyaw')
        self.chestyaw.sin1.value = -1
        plug (self.oscillatorYaw.vectorSout, self.chestyaw.sin2)

        
        self.stack1 = Stack_of_vector('Stack1')
        self.stack1.sin1.value = (0.0,)*6
        plug ( self.oscillatorYaw.vectorSout, self.stack1.sin2 )
        self.stack1.selec1(0,6)
        self.stack1.selec2(0,1)

        self.stack2 = Stack_of_vector('Stack2')
        plug ( self.stack1.sout, self.stack2.sin1 )
        self.stack2.sin2.value = (0.0,)
        self.stack2.selec1(0,7)
        self.stack2.selec2(0,1)

        self.stack3 = Stack_of_vector('Stack3')
        plug ( self.stack2.sout, self.stack3.sin1 )
        plug ( self.oscillatorPitch.vectorSout, self.stack3.sin2 )
        self.stack3.selec1(0,8)
        self.stack3.selec2(0,1)

        self.stack4 = Stack_of_vector('Stack4')
        plug ( self.stack3.sout, self.stack4.sin1 )
        self.stack4.sin2.value=(0.0,0.0,0.0)
        self.stack4.selec1(0,9)
        self.stack4.selec2(0,3)

        self.stack5 = Stack_of_vector('Stack5')
        plug ( self.stack4.sout, self.stack5.sin1 )
        plug ( self.oscillatorYaw.vectorSout, self.stack5.sin2 )
        self.stack5.selec1(0,12)
        self.stack5.selec2(0,1)

        self.stack6 = Stack_of_vector('Stack6')
        plug ( self.stack5.sout, self.stack6.sin1 )
        self.stack6.sin2.value = (0.0,)
        self.stack6.selec1(0,13)
        self.stack6.selec2(0,1)

        self.stack7 = Stack_of_vector('Stack7')
        plug ( self.stack6.sout, self.stack7.sin1 )
        plug ( self.minuspitch.sout, self.stack7.sin2 )
        self.stack7.selec1(0,14)
        self.stack7.selec2(0,1)

        self.stack8 = Stack_of_vector('Stack8')
        plug ( self.stack7.sout, self.stack8.sin1 )
        self.stack8.sin2.value=(0.0,0.0,0.0)
        self.stack8.selec1(0,15)
        self.stack8.selec2(0,3)

        self.stack9 = Stack_of_vector('Stack9')
        plug ( self.stack8.sout, self.stack9.sin1 )
        plug ( self.chestyaw.sout, self.stack9.sin2 )
        self.stack9.selec1(0,18)
        self.stack9.selec2(0,1)

        self.stack10 = Stack_of_vector('Stack10')
        plug ( self.stack9.sout, self.stack10.sin1 )
        self.stack10.sin2.value = (0.0,)*17
        self.stack10.selec1(0,19)
        self.stack10.selec2(0,17) 

        self.postureRef =  self.stack10.sout     
        
               


    def initTaskPosture(self):
        # --- LEAST NORM
        weight_ff        = 0
        weight_leg       = 3
        weight_knee      = 5
        weight_chest     = 1
        weight_chesttilt = 10
        weight_head      = 0.3
        weight_arm       = 1

        weight = np.diag( (weight_ff,)*6 + (weight_leg,)*12 + (weight_chest,)*2 + (weight_head,)*2 + (weight_arm,)*14)
        weight[9,9] = weight_knee
        weight[15,15] = weight_knee
        weight[19,19] = weight_chesttilt
        #weight = weight[6:,:]

        self.featurePosture.jacobianIN.value = matrixToTuple(weight)
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        mask = '1'*36
        # mask = 6*'0'+12*'0'+4*'1'+14*'0'
        # mask = '000000000000111100000000000000000000000000'
        # robot.dynamic.displaySignals ()
        # robot.dynamic.Jchest.value
        self.features['posture'].selec.value = mask

    def initTaskGains(self, setup = "medium"):
        if setup == "medium":
            self.gains['balance'].setConstant(10)
            self.gains['trunk'].setConstant(10)
            self.gains['right-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.gains['left-wrist'].setByPoint(4,0.2,0.01,0.8)
            self.taskHalfStitting.gain.setByPoint(2,0.2,0.01,0.8)
         
    # --- SOLVER ----------------------------------------------------------------

    def push(self,task,feature=None,keep=False):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName not in toList(self.sot):
            self.sot.push(taskName)
        if taskName!="posture" and "posture" in toList(self.sot):
            self.sot.down("posture")
        if keep: feature.keep()

    def rm(self,task):
        if isinstance(task,str): taskName=task
        elif "task" in task.__dict__:  taskName=task.task.name
        else: taskName=task.name
        if taskName in toList(self.sot): self.sot.remove(taskName)

    # --- DISPLAY -------------------------------------------------------------
    def initDisplay(self):
        self.robot.device.viewer.updateElementConfig('red',[0,0,-1,0,0,0])
        self.robot.device.viewer.updateElementConfig('yellow',[0,0,-1,0,0,0])

    def updateDisplay(self):
        '''Display the various objects corresponding to the calcul graph. '''
        None

    # --- TRACES -----------------------------------------------------------
    def withTraces(self):
        if self.tracesRealTime:
            self.robot.tracerSize = 2**26
            self.robot.initializeTracer()
        else:
            self.robot.tracer = Tracer('trace')
            self.robot.device.after.addSignal('{0}.triger'.format(self.robot.tracer.name))
        self.robot.tracer.open('/tmp/','','.dat')
        #self.robot.tracer.add( self.taskRH.task.name+'.error','erh' )
        
    def stopTracer(self):
        self.robot.stopTracer()

    def dumpTracer(self):
        self.robot.tracer.dump()
    
    def startTracer(self):
        self.robot.startTracer()

    # --- RUN --------------------------------------------------------------
    def initialStack(self):
        self.sot.clear()
        self.push(self.tasks['balance'])
        self.push(self.taskTrunk)
        #self.push(self.taskPosture)

    def moveToInit(self):
        '''Go to initial pose.'''
        self.sot.clear()
        self.featurePostureDes.errorIN.value = (0,)*36;
        #gotoNd(self.taskRH,(0.3,-0.2,1.1,0,-pi/2,0),'111001')
        #gotoNd(self.taskLH,(0.3,0.2,1.1,0,-pi/2,0),'111001')
        change6dPositionReference(self.taskRH,self.features['right-wrist'],\
                                    self.gains['right-wrist'],\
                                    (0.3,-0.3,1.1,0,-np.pi/2,0),'111111')
        self.push(self.taskRH)
        if self.twoHands:
            change6dPositionReference(self.taskLH,self.features['left-wrist'],\
                                    self.gains['left-wrist'],\
                                     (0.3,0.3,1.1,0,-np.pi/2,0),'111111')
            self.push(self.taskLH)
            
        self.push(self.tasks['trunk'])
        self.push(self.tasks['posture'])

        
        
        None

    def goHalfSitting(self):
        '''End of application, go to final pose.'''
        self.gains['posture'].gain.setByPoint(4,0.2,0.01,0.8)  
        self.featurePostureDes.errorIN.value = self.robot.halfSitting
        self.sot.clear()
        self.push(self.tasks['balance'])
        self.push(self.taskPosture)
        

    # --- SEQUENCER ---
    seqstep = 0
    def nextStep(self,step=None):
        if step!=None: self.seqstep = step
        if self.seqstep==0:
            self.moveToInit()
        elif self.seqstep==1:
            self.startCompensate()
        elif self.seqstep==2:
            self.startOcillation()
        elif self.seqstep==3:
            self.stopOcillation()
        elif self.seqstep==4:
            self.goHalfSitting()
        self.seqstep += 1
        
    def __add__(self,i):
        self.nextStep()


    # COMPENATION ######################################

    def initTaskCompensate(self):
        # The constraint is:
        #    cMhref !!=!! cMh = cMcc ccMh
        # or written in ccMh
        #    ccMh !!=!! ccMc cMhref

        # c : central frame of the robot
        # cc : central frame for the controller  (without the flexibility)
        # cMcc= flexibility
        # ccMc= flexibility inverted

        self.transformerR = MovingFrameTransformation('tranformation_right')

        self.ccMc = self.transformerR.gMl # inverted flexibility
        self.cMrhref = self.transformerR.lM0 # reference position in the world control frame
        # You need to set up the inverted flexibility : plug( ..., self.ccMc)
        # You need to set up a reference value here: plug( ... ,self.cMhref)

        self.ccVc = self.transformerR.gVl # inverted flexibility velocity
        self.cVrhref = self.transformerR.lV0 # reference velocity in the world control frame
        # You need to set up the inverted flexibility velocity : plug( ..., self.ccVc)
        # You need to set up a reference velocity value here: plug( ... ,self.cVhref)

        self.ccMrhref = self.transformerR.gM0 # reference matrix homo in the control frame
        self.ccVrhref = self.transformerR.gV0
        
        ######

        if self.twoHands:
            self.transformerL = MovingFrameTransformation('tranformation_left')

            self.sym = Multiply_of_matrixHomo('sym')

            self.sym.sin1.value =((1, 0, 0, 0), (0, -1, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1))
            plug (self.ccMrhref,self.sym.sin2)
            


            self.symVel = Multiply_matrix_vector('symvel')
            self.symVel.sin1.value =((1,0,0,0,0,0),(0,-1,0,0,0,0),(0,0,0,1,0,0),(0,0,0,1,0,0),(0,0,0,0,-1,0),(0,0,0,0,0,1))
            plug (self.ccVrhref,self.symVel.sin2)
            
        

            self.cMrhref.value = (matrixToTuple(np.diag([1,1,1,1])))
            self.cVrhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)

    def startCompensate(self):


        '''Start to compensate for the hand movements.'''
        self.cMrhref.value = self.robot.dynamic.signal('right-wrist').value
        self.cVrhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        
        plug(self.ccMrhref,self.features['right-wrist'].reference)
        plug(self.ccVrhref,self.features['right-wrist'].velocity)
        
        self.gains['right-wrist'].setByPoint(4,0.2,0.01,0.8)        
        self.tasks['right-wrist'].setWithDerivative (True)
        self.features['right-wrist'].frame('desired')

        print matrix(self.cMrhref.value)

        if self.twoHands:
            self.gains['left-wrist'].setByPoint(4,0.2,0.01,0.8)
            
            plug (self.sym.sout,self.features['left-wrist'].reference)
            plug (self.symVel.sout,self.features['left-wrist'].velocity)
            self.features['left-wrist'].selec.value='000111'
            self.tasks['left-wrist'].setWithDerivative (True)
            self.features['left-wrist'].frame('desired')
        
            #self.tasks['gaze'].setWithDerivative (True)            
        
        #######

        #self.cMlhref.value = self.robot.dynamic.lh.value
        #self.cVlhref.value = (0.0 , 0.0 , 0.0 , 0.0 , 0.0 , 0.0)
        #print matrix(self.cMlhref.value)

        ######
    
    def startOcillation(self):
        self.stack10.sin2.value = self.robot.dynamic.position.value[19:36]
        self.sot.clear()
        self.gains['posture'].gain.value=10
        self.push(self.tasks['posture'])
        plug (self.postureRef,self.featurePostureDes.errorIN)
        self.oscillatorPitch.setActivated(True)
        self.oscillatorYaw.setActivated(True)

    def stopOcillation(self):
        self.oscillatorPitch.setActivated(False)
        self.oscillatorYaw.setActivated(False)
        
