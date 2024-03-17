%% Clear workspace
close all
clear all
clc

% Robot = robotConfigInv();
% 
% Robot = inverseKinematics(Robot);
% 
% plot([Robot.A(1) Robot.C(1)], [Robot.A(2) Robot.C(2)], 'or');
% axis([0 10 0 10]);
% hold on
% 
% Robot = forwardKinematics(Robot)
% plot([Robot.A(1) Robot.B(1) Robot.C(1)], [Robot.A(2) Robot.B(2) Robot.C(2)]);

%% Initialization of the robot and environment
[Robot, Environment] = robotInit();

%%
% calculate L1,L2, theta1, theta2 from inverse kinematics

%plot goal position (red)
% distance = Robot.L1 + Robot.L2 + 1;
% while (distance >= (Robot.L1 + Robot.L2))
%     title(sprintf('Insert robot goal position C'));
%     [Robot.goal(1),Robot.goal(2)] = ginput(1);   %read one point
%     plot(Robot.goal(1),Robot.goal(2),'*r');
%     distance = dist(Robot.vertices(:,1),Robot.goal);
% end

Robot.vertices(:, 3) = Robot.goal;
Robot = inverseKinematics(Robot);
Robot = forwardKinematics(Robot);
plot(Robot.vertices(1, 1:3), Robot.vertices(2, 1:3), '--r');
plot(Robot.vertices(1, [1, 4, 3]), Robot.vertices(2, [1, 4, 3]), '--r');
