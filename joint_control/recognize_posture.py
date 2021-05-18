'''In this exercise you need to use the learned classifier to recognize current posture of robot

* Tasks:
    1. load learned classifier in `PostureRecognitionAgent.__init__`
    2. recognize current posture in `PostureRecognitionAgent.recognize_posture`

* Hints:
    Let the robot execute different keyframes, and recognize these postures.

'''


from angle_interpolation import AngleInterpolationAgent
from keyframes import hello
import pickle
from os import listdir
import numpy as np
from keyframes import leftBackToStand

pose = listdir('robot_pose_data')


class PostureRecognitionAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(PostureRecognitionAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.posture = 'unknown'
        self.posture_classifier = pickle.load(open('robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def think(self, perception):
        self.posture = self.recognize_posture(perception)
        return super(PostureRecognitionAgent, self).think(perception)

    def recognize_posture(self, perception):
        posture = 'unknown'
        # YOUR CODE HERE
        
        postures = []
        
        #postures.append(perception.joint['LShoulderPitch'])
        #postures.append(perception.joint['LShoulderRoll'])
        #postures.append(perception.joint['LElbowYaw'])
        #postures.append(perception.joint['LElbowRoll'])
        
        #postures.append(perception.joint['LWristYaw'])
        #postures.append(perception.joint['LHand'])
        
        postures.append(perception.joint['LHipYawPitch']) 
        postures.append(perception.joint['LHipRoll']) 
        postures.append(perception.joint['LHipPitch'])
        postures.append(perception.joint['LKneePitch'])
        
        #postures.append(perception.joint['LAnklePitch'])
        #postures.append(perception.joint['LAnkleRoll'])
        
        #postures.append(perception.joint['RShoulderPitch'])
        #postures.append(perception.joint['RShoulderRoll'])
        #postures.append(perception.joint['RElbowYaw'])
        #postures.append(perception.joint['RElbowRoll'])
        
        #postures.append(perception.joint['RWristYaw'])
        #postures.append(perception.joint['RHand'])
        
        postures.append(perception.joint['RHipYawPitch'])
        postures.append(perception.joint['RHipRoll'])
        postures.append(perception.joint['RHipPitch'])
        postures.append(perception.joint['RKneePitch'])
        
        #postures.append(perception.joint['RAnklePitch'])
        #postures.append(perception.joint['RAnkleRoll'])
        
        postures.append(perception.imu[0])
        postures.append(perception.imu[1])
        
        posture = pose[self.posture_classifier.predict(np.array(postures).reshape(1,-1))[0]]

        #print(posture)
		
        return posture

if __name__ == '__main__':
    agent = PostureRecognitionAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    #agent.keyframes = leftBackToStand()
    agent.run()
