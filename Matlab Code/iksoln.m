% Define the UR5 robot model
% Assuming 'ur' is already defined as a rigidBodyTree in the workspace

% Set up Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', ur); % 'ur' is the robot model
ik.SolverParameters.MaxIterations = 10000;
ik.SolverParameters.GradientTolerance = 1e-6;
ik.SolverParameters.SolutionTolerance = 1e-6;

% Define the end-effector name
endEffector = ur.BodyNames{end}; % Assuming the last body is the end-effector

% Initial guess for the joint positions
initialGuess = homeConfiguration(ur);


N = size(waypoint, 2); % Number of waypoints from the input

% Allocate space for the solution: [6xN] matrix to store joint positions
jointPositions = zeros(6, N);

% Define the orientation for the end-effector (you can modify this based on your requirement)
desiredOrientation = eul2quat([0 0 0]); % Zero orientation (roll, pitch, yaw)

% Loop through each waypoint and solve inverse kinematics
for i = 1:N
    % Get the target Cartesian position from the input
    targetPosition = waypoint(:, i);
    
    % Form the desired pose as a homogeneous transformation matrix
    desiredPose = trvec2tform(targetPosition') * quat2tform(desiredOrientation);

    % Solve inverse kinematics to get joint positions for this waypoint
    [configSol, ~] = ik(endEffector, desiredPose, ones(1, 6), initialGuess);
    
    % Extract joint positions from configSol
    jointPos = zeros(6, 1); % Pre-allocate a 6x1 vector for joint positions
    for j = 1:6
        jointPos(j) = configSol(j); % Access each joint position
    end
    
    % Store the solution (joint positions)
    jointPositions(:, i) = jointPos;
    
    % Update initial guess for next iteration
    initialGuess = configSol;
end
%zerocol= zeros(1,6);
%jointPositions= [zerocol ; jointPositions];

% Output: [6xN] joint positions
disp(jointPositions);
