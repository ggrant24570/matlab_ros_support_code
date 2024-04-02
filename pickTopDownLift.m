% Have this new script call moveTopDownCan and then after that create 
% another function that builds on what you did in section B but now lifts 
% the arm instead of making it descend. It can go up to the original height
% from which it descended
function [x,y,z,r,p,h] = pickTopDownLift

pickTopDownCan

models = getModels; 

model_name = models.ModelNames{26}; % 'rCan3'

[gripper_wrt_base_pose, object_wrt_base_pose] = get_robot_object_pose_wrt_base_link(model_name);

trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                          'control_msgs/FollowJointTrajectory',...
                          'DataFormat', 'struct') 
trajGoal = rosmessage(trajAct)

trajAct.FeedbackFcn = []; 

jointSub = rossubscriber("/joint_states")

jointStateMsg = jointSub.LatestMessage

UR5e = loadrobot('universalUR5e', DataFormat="row")

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); 

ikWeights = [0.25 0.25 0.25 0.1 0.1 .1];

jointStateMsg = receive(jointSub,3)

initialIKGuess = homeConfiguration(UR5e)

initialIKGuess(1) = jointStateMsg.Position(4);  % Shoulder Pan
initialIKGuess(2) = jointStateMsg.Position(3);  % Shoulder Tilt
initialIKGuess(3) = jointStateMsg.Position(1);  % Elbow
initialIKGuess(4) = jointStateMsg.Position(5);  % W1
initialIKGuess(5) = jointStateMsg.Position(6);  % W2
initialIKGuess(6) = jointStateMsg.Position(7);  % W3

gripperX = 0.8;
gripperY = -0.0355;
gripperZ = 0.4;

gripperTranslation = [gripperY gripperX gripperZ];
gripperRotation    = [-pi/2 -pi 0]; %  [Z Y Z] radians


tform = eul2tform(gripperRotation); % ie eul2tr call
tform(1:3,4) = gripperTranslation'; % set translation in homogeneous transform

[configSoln, solnInfo] = ik('tool0',tform,ikWeights,initialIKGuess)

UR5econfig = [configSoln(3)... 
              configSoln(2)...
              configSoln(1)...
              configSoln(4)...
              configSoln(5)...
              configSoln(6)]

trajGoal = packTrajGoal(UR5econfig,trajGoal)

sendGoalAndWait(trajAct,trajGoal)

x = trajGoal.Trajectory.Points.Positions(1);
y = trajGoal.Trajectory.Points.Positions(2);
z = trajGoal.Trajectory.Points.Positions(3);
r = trajGoal.Trajectory.Points.Positions(4);
p = trajGoal.Trajectory.Points.Positions(5);
h = trajGoal.Trajectory.Points.Positions(6);

end