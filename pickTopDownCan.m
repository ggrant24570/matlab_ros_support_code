% Have this new script call moveTopDownCan and then after that create 
% another function that calls code from notebook 09_actions_UR_robot.mlx
% section 2.4 to close fingers around rCan3.
function [x,y,z,r,p,h] = pickTopDownCan

moveTopDownCan;

grip_client = rosactionclient('/gripper_controller/follow_joint_trajectory',...
                              'control_msgs/FollowJointTrajectory',...
                              'DataFormat', 'struct');
gripGoal    = rosmessage(grip_client);
gripPos     = 0.22; %0.22 is grab
gripGoal    = packGripGoal(gripPos,gripGoal);
pause(3);
sendGoal(grip_client,gripGoal)

x = gripGoal.Trajectory.Points.Positions(1);
y = gripGoal.Trajectory.Points.Positions(2);
z = gripGoal.Trajectory.Points.Positions(3);
r = gripGoal.Trajectory.Points.Positions(4);
p = gripGoal.Trajectory.Points.Positions(5);
h = gripGoal.Trajectory.Points.Positions(6);

end