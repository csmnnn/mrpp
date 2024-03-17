function Robot = inverseKinematics(Robot)
%INVERSEKINEMATICS Inverse kinematics function.
%   INVERSEKINEMATICS(ROBOT) for a structure ROBOT, computes the rotation
%   angles of the links θ1 and θ2, given the fixed position of robot A, the 
%   position of the effector C, and the lengths of the two links L1 and L2.
%   The result is stored in the structure given.

    AC = norm(Robot.vertices(:, 1) - Robot.vertices(:, 3)); % Distance from base (A) to effector (C)
    xD = Robot.vertices(1, 3) - Robot.vertices(1, 1); % X coordinate of point D

    Robot.theta(1, 1) = acos(xD/AC) - ...
        acos((AC^2 + Robot.L1^2 - Robot.L2^2) / (2 * AC * Robot.L1)); % θ1 for the first configuration
    Robot.theta(1, 2) = acos(-(Robot.L1^2 + Robot.L2^2 - AC^2) / ...
        (2 * Robot.L1 * Robot.L2)); % θ2 for the first configuration

    Robot.theta(2, 1) = 2 * acos((AC^2 + Robot.L1^2 - Robot.L2^2) / ...
        (2 * AC * Robot.L1)) + Robot.theta(1, 1); % θ1 for the second configuration (symmetrical with the first)
    Robot.theta(2, 2) = -Robot.theta(1, 2); % θ2 for the second configuration (symmetrical with the first)

    Robot.theta = mod(Robot.theta, 2*pi); % Making sure angles are between 0 and 2π
end
