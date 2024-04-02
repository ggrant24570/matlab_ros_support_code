% This m-file will execute 2 independent motions. The first one is to move 
% with a constant height directly on top of the rCan3. The second will 
% produce a straight top-down move such that the end-effector hugs the top
% of rCan3 ready to pick it up.
function [x,y,z,r,p,h] = moveTopDownCan

%clear
masterhostIP = "192.168.248.128";
rosshutdown;
rosinit(masterhostIP)

disp('Going home...');
goHome('qr');    % moves robot arm to a qr or qz start config

disp('Resetting the world...');
resetWorld      % reset models through a gazebo service

models = getModels; 

model_name = models.ModelNames{26}; % 'rCan3'

[gripper_wrt_base_pose, object_wrt_base_pose] = get_robot_object_pose_wrt_base_link(model_name);
% - mat_R_T_G [4x4] double - transformation from robot base_link to tip
% - mart_R_T_M [4x4] double -  transformation from robot base_link to obj

%robot = loadrobot("universalUR5e",DataFormat="row"); 

trajAct = rosactionclient('/pos_joint_traj_controller/follow_joint_trajectory',...
                          'control_msgs/FollowJointTrajectory',...
                          'DataFormat', 'struct') 

trajGoal = rosmessage(trajAct)

trajAct.FeedbackFcn = []; 

jointSub = rossubscriber("/joint_states")

jointStateMsg = jointSub.LatestMessage

UR5e = loadrobot('universalUR5e', DataFormat="row")

tform=UR5e.Bodies{3}.Joint.JointToParentTransform;    % shoulder_pan_joint
UR5e.Bodies{3}.Joint.setFixedTransform(tform*eul2tform([pi/2,0,0]));

tform=UR5e.Bodies{4}.Joint.JointToParentTransform;      % shoulder_lift_joint
UR5e.Bodies{4}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

tform=UR5e.Bodies{7}.Joint.JointToParentTransform;      % wrist_2_joint
UR5e.Bodies{7}.Joint.setFixedTransform(tform*eul2tform([-pi/2,0,0]));

ik = inverseKinematics("RigidBodyTree",UR5e); % Create Inverse kinematics solver

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
%gripperY = -0.04;
gripperY = -0.036;
%gripperZ = 0.13;
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
%%%%%%%%%%%%%%%%%%%%%%%%
gripperX = 0.8;
%gripperY = -0.04;
gripperY = -0.036;
gripperZ = 0.13;

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