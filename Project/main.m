%
%
% Template to visualize in V-REP the inverse kinematics algorithm developed
%          for a 7-DOF robot of the family Kuka LWR or IIWA
%
% Read Instructions.odt first !
% 
% Do not modify any part of this file except the strings within
%    the symbols << >>
%
% G. Antonelli, Sistemi Robotici, fall 2014


function [t, q, q_act] = main

    porta = 19997;          % default V-REP port
    
    
    isOptimized = false; % maximize manipulability

    K = diag([50*[1 1 1], 10*[1 1 1]]);

    % initialize
    tf = 2; % time duration
    Ts = 1e-3; % sampling time
    n = 7; % number of joints
    t = 0:Ts:1.5*tf; 
    N = length(t);
    x_i = [0.32 -0.55 0.31]'; % initial position
    x_f = [-0.5 -0.1 0.3]'; % final position

    xd  = zeros(3,N);
    q   = zeros(n,N);

    % orient_i = [0 0 0 1]'; % give some inital orientation 
    % q(:,1) = getInitialJointConfiguration(xi, orient_i);

    q(:,1) = [1.56489  1.38241  -2.07832  1.24803  -1.20479  1.97508  0.45348]';
    dq  = zeros(n,N);
    quat = zeros(4,N);
    error_pos = zeros(3,N);
    error_quat = zeros(3,N);
    error = zeros(6,N);

    dx_c = [1.5 1.6 1.8]'.*abs(x_f-x_i)/tf; % cruise

    k_a = 10000;
    dq_a = zeros(n,1);

    % <<
    %
    % Put here any initialization code: DH table, gains, final position,
    % cruise velocity, etc.
    %
    % >>
    
    clc
    fprintf('----------------------');
    fprintf('\n simulation started ');
    fprintf('\n trying to connect...\n');
    [clientID, vrep ] = StartVrep(porta);
    
    handle_joint = my_get_handle_Joint(vrep,clientID);      % handle to the joints
    my_set_joint_target_position(vrep, clientID, handle_joint, q(:,1)); % first move to q0
    q_act(:,1) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     

    % main simulation loop
    for i=1:N
        % get the intermediate desired position
        [xd(:,i),dx(:,i),ddx(:,i)] = trapezoidal(x_i,x_f,dx_c,tf,t(i));

        % direct kinematics
        T = kuka_directkinematics(q(:,i));

        % current position
        x(:,i) = T(1:3,4);

        % current orientation (in quaternions)
        quat(:,i) = Rot2Quat(T(1:3,1:3));

        % desired orientation (same as inital orientation)
        quat_d(:,i) = quat(:,i);

        % Jacobian (6x7)
        J = kuka_J(q(:,i)); 

        % Inverse kinematics algorithm

        % position error
        error_pos(:,i) = xd(:,i) - x(:,i); 

        % orientation error
        error_quat(:,i) = QuatError(quat_d(:,i),quat(:,i));

        % current error 
        error(:,i) = [error_pos(:,i);error_quat(:,i)];

        % compute joint displacement needed for reducing the error
        if isOptimized
          dq_a = get_dqa(q(:,i), k_a);
        end
        particular_sol = pinv(J) * K * error(:,i);
        homogenuous_sol = (eye(n) - pinv(J) * J) * dq_a;
        dq(:,i) = particular_sol + homogenuous_sol;

        if i<N
            % update joint configuration
            q(:,i+1) = q(:,i) + Ts*dq(:,i);
        end
        my_set_joint_target_position(vrep, clientID, handle_joint, q(:,i));
        q_act(:,i) = my_get_joint_target_position(clientID,vrep,handle_joint,n);% get the actual joints angles from v-rep     
    end

    DeleteVrep(clientID, vrep); 
            
end

% constructor
function [clientID, vrep ] = StartVrep(porta)

    vrep = remApi('remoteApi');   % using the prototype file (remoteApiProto.m)
    vrep.simxFinish(-1);        % just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1',porta,true,true,5000,5);% start the simulation
    
    if (clientID>-1)
        disp('remote API server connected successfully');
    else
        disp('failed connecting to remote API server');   
        DeleteVrep(clientID, vrep); %call the destructor!
    end
    % to change the simulation step time use this command below, a custom dt in v-rep must be selected, 
    % and run matlab before v-rep otherwise it will not be changed 
    % vrep.simxSetFloatingParameter(clientID, vrep.sim_floatparam_simulation_time_step, 0.002, vrep.simx_opmode_oneshot_wait);
    
end  

% destructor
function DeleteVrep(clientID, vrep)

    vrep.simxPauseSimulation(clientID,vrep.simx_opmode_oneshot_wait); % pause simulation
%   vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot_wait); % stop simulation
    vrep.simxFinish(clientID);  % close the line if still open
    vrep.delete();              % call the destructor!
    disp('simulation ended');
    
end

function my_set_joint_target_position(vrep, clientID, handle_joint, q)
           
    [m,n] = size(q);
    for i=1:n
        for j=1:m
            err = vrep.simxSetJointTargetPosition(clientID,handle_joint(j),q(j,i),vrep.simx_opmode_oneshot);
            if (err ~= vrep.simx_error_noerror)
                fprintf('failed to send joint angle q %d \n',j);
            end
        end
    end
    
end

function handle_joint = my_get_handle_Joint(vrep,clientID)

    [~,handle_joint(1)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint1',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(2)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint2',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(3)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint3',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(4)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint4',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(5)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint5',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(6)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint6',vrep.simx_opmode_oneshot_wait);
    [~,handle_joint(7)] = vrep.simxGetObjectHandle(clientID,'LBR_iiwa_7_R800_joint7',vrep.simx_opmode_oneshot_wait);

end

function my_set_joint_signal_position(vrep, clientID, q)
           
    [~,n] = size(q);
    
    for i=1:n
        joints_positions = vrep.simxPackFloats(q(:,i)');
        [err]=vrep.simxSetStringSignal(clientID,'jointsAngles',joints_positions,vrep.simx_opmode_oneshot_wait);

        if (err~=vrep.simx_return_ok)   
           fprintf('failed to send the string signal of iteration %d \n',i); 
        end
    end
    pause(8);% wait till the script receives all data, increase it if dt is too small or tf is too high
    
end


function angle = my_get_joint_target_position(clientID,vrep,handle_joint,n)
    
    for j=1:n
         vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_streaming);
    end

    pause(0.05);

    for j=1:n          
         [err(j),angle(j)]=vrep.simxGetJointPosition(clientID,handle_joint(j),vrep.simx_opmode_buffer);
    end

    if (err(j)~=vrep.simx_return_ok)   
           fprintf(' failed to get position of joint %d \n',j); 
    end

end

