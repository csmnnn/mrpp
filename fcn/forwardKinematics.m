function Robot = forwardKinematics(Robot)
%FORWARDKINEMATICS Forward kinematics function.
%   FORWARDKINEMATICS(ROBOT) for a structure ROBOT, computes the position
%   of joint B and the position of the effector C, given the lengths of the 
%   two links L1 and L2, the position of the base of the robot A, and the
%   rotation angles of the links θ1 and θ2.
%   The result is store in the structure given.

    Robot.vertices(:, 2) = [Robot.vertices(1, 1) + Robot.L1*cos(Robot.theta(1, 1)) ...
        Robot.vertices(2, 1) + Robot.L1*sin(Robot.theta(1, 1))]; % Position of the joint B

    Robot.vertices(:, 3) = [Robot.vertices(1, 2) + Robot.L2*cos(Robot.theta(1, 1) + Robot.theta(1, 2)) ...
        Robot.vertices(2, 2) + Robot.L2*sin(Robot.theta(1, 1) + Robot.theta(1, 2))]; % Position of the effector C

    Robot.vertices(:, 4) = [Robot.vertices(1, 1) + Robot.L1*cos(Robot.theta(2,1)) ...
        Robot.vertices(2, 1) + Robot.L1*sin(Robot.theta(2, 1))]; % Symmetrical position of the joint B
end
