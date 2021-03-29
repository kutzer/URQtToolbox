classdef URQt < matlab.mixin.SetGet % Handle
    % URQt handle class for creating a designated UR hardware connection.
    %
    %   obj = URQt creates a hardware connection object for a specific UR
    %
    % URQt Methods
    %   Initialize  - Initialize the URQt object.
    %   WaitForMove - Wait until UR completes specified movement.
    %   Home        - Move URQt to home joint configuration.
    %   Stow        - Move URQt to stow joint configuration.
    %   Zero        - Move URQt to zero joint configuration.
    %   Undo        - Return URQt to previous joint configuration.
    %   get         - Query properties of the URQt object.
    %   set         - Update properties of the URQt object.
    %   delete      - Delete the URQt object and all attributes.
    %
    % URQt Properties
    % -Qt Connection
    %   IP          - String containing IP address for Qt connection
    %   Port        - Integer specifying port for Qt connection
    %   Client      - TCP client object for Qt connection
    %
    % -Universal Robot Model
    %   URmodel     - string argument defining model of Universal Robot
    %
    % -DH Table
    %   DHtable     - nx4 array defining the DH table of the Universal
    %                 Robot
    %
    % -Joint Values
    %   Joints      - 1x6 array containing joint values (radians)
    %   Joint1      - scalar value containing joint 1 (radians)
    %   Joint2      - scalar value containing joint 2 (radians)
    %   Joint3      - scalar value containing joint 3 (radians)
    %   Joint4      - scalar value containing joint 4 (radians)
    %   Joint5      - scalar value containing joint 5 (radians)
    %   Joint6      - scalar value containing joint 6 (radians)
    %
    % -End-effector pose
    %   Pose        - 4x4 rigid body transform defining the end-effector
    %                 pose relative to the world frame (linear units are
    %                 defined in millimeters)
    %   Task        - 1x6 array of [x,y,z,r1,r2,r3] matching the task
    %                 variables used by UR with the exception of linear 
    %                 units (x,y,z) that are specified in millimeters 
    %
    % -Tool pose
    %   ToolPose    - 4x4 rigid body transform defining the tool pose
    %                 relative to the world frame (linear units are defined
    %                 in millimeters)
    %   ToolTask    - 1x6 array of [x,y,z,r1,r2,r3] matching the task
    %                 variables used by UR with the exception of linear 
    %                 units (x,y,z) that are specified in millimeters
    %
    % -Frame Definitions
    %   FrameT      - Tool Frame (transformation relative to the
    %                 End-effector Frame)
    %
    % Example:
    %
    %       % Create, initialize, and visualize
    %       ur3e = URQt;              % Create URQt object
    %       ur3e.Initialize('UR3e');  % Designate as UR3e
    %
    %       % Send hardware to stow configuration
    %       ur3e.Stow;
    %       ur3e.WaitForMove;
    %       % Send simulation to home configuration
    %       ur3e.Home;
    %       ur3e.WaitForMove;
    %       % Send simulation to specified joint configuration
    %       ur3e.Joints = 2*pi*rand(1,6);
    %       ur3e.WaitForMove;
    %
    % See also
    %
    %   M. Kutzer 26Mar2021, USNA
    
    % Updates

    
    % --------------------------------------------------------------------
    % General properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='private')
        IP          % String containing IP address for Qt connection
        Port        % Integer specifying port for Qt connection
        Client      % TCP client object for Qt connection      
    end
    
    properties(GetAccess='public', SetAccess='private')
        URmodel     % Specified type of Universal Robot Manipulator
    end
    
    properties(GetAccess='public', SetAccess='public')
        Joints      % 1x6 array containing joint values (radians)
        Pose        % 4x4 homogeneous transform representing the end-effector pose relative to the world frame
        Task        % 1x6 array containing end-effector pose in task space
        ToolPose    % 4x4 homogeneous transform representing the tool pose relative to the world frame
        ToolTask    % 1x6 array containing tool pose in task space
    end % end properties
    
    properties(GetAccess='public', SetAccess='private')
        DHtable     % DH table associated with robot
    end % end properties
    
    properties(GetAccess='public', SetAccess='public')
        Frame0      % Frame 0 (transformation relative to World/Axes Frame)
    end % end properties
    
    properties(GetAccess='public', SetAccess='private', Hidden=true)
        Frame1      % Frame 1 (transformation relative to Frame 0)
        Frame2      % Frame 2 (transformation relative to Frame 1)
        Frame3      % Frame 3 (transformation relative to Frame 2)
        Frame4      % Frame 4 (transformation relative to Frame 3)
        Frame5      % Frame 5 (transformation relative to Frame 4)
        Frame6      % Frame 6 (transformation relative to Frame 5)
        FrameE      % End-effector Frame (transformation relative to Frame 6)
    end % end properties
    
    properties(GetAccess='public', SetAccess='public')
        FrameT      % Tool Frame (transformation relative to the End-effector Frame)
    end % end properties
    
    properties(GetAccess='public', SetAccess='public', Hidden=true)
        Joint1      % Joint 1 value (radians)
        Joint2      % Joint 2 value (radians)
        Joint3      % Joint 3 value (radians)
        Joint4      % Joint 4 value (radians)
        Joint5      % Joint 5 value (radians)
        Joint6      % Joint 6 value (radians)
    end % end properties
    
    % --------------------------------------------------------------------
    % Internal properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='private', Hidden=true)
        QtPath      % Path of Qt executable
        QtEXE       % Name of Qt executable
        Joints_Old	% Previous joint configuration (used with Undo)
    end
    
    % --------------------------------------------------------------------
    % Constructor/Destructor
    % --------------------------------------------------------------------
    methods(Access='public')
        function obj = URQt
            % Create URQt Object
            
            % Define Qt executable path
            dName = 'urqt';
            fName = 'URQtSupport';
            obj.QtPath = fullfile(matlabroot,'toolbox',dName,fName);
            
            % Define Qt executable name
            obj.QtEXE = 'Roswell.exe';
        end
        
        function delete(obj)
            % Object destructor
            
        end
    end % end methods
    
    % --------------------------------------------------------------------
    % Initialization
    % --------------------------------------------------------------------
    methods(Access='public')
        function Initialize(obj,varargin)
            % Initialize initializes a Universal Robot simulation
            %
            % Initialize(obj)
            %
            % Initialize(obj,URmodel)
            %
            % Initialize(obj,URmodel,IP)
            %
            % Initialize(obj,URmodel,IP,Port)
            
            % Initialize Qt Interface
            fprintf('Starting Qt interface...');
            if ~obj.isQtRunning
                if ~obj.existQt
                    fprintf('"%s" not found, FAILED\n',obj.QtEXE);
                    return
                end
                % Start Qt interface
                obj.startQt;
                % Check if Qt interface is running
                t0 = tic;
                tf = 30;
                while ~obj.isQtRunning
                    % Wait for server to start
                    t = toc(t0);
                    if t > tf
                        fprintf('TIMEOUT\n');
                        warning('Server did not start successfully.');
                        return
                    end
                end
                fprintf('SUCCESS\n');
            else
                fprintf('SKIPPED\n');
                fprintf('\tQt interface is already running.\n');
            end
            
            % Initialize IP and Port
            % TODO - allow user to specify IP and Port
            obj.IP = '127.0.0.1';
            obj.Port = 8897;
            % TODO - specify and use terminator
            % TODO - specify terminator callback function
            fprintf('Initializing TCP Client: IP %s, Port %d...',...
                obj.IP,obj.Port);
            obj.Client = tcpclient(obj.IP,obj.Port);
            fprintf('SUCCESS\n');
        end
    end % end methods
    
    % --------------------------------------------------------------------
    % General Use
    % --------------------------------------------------------------------
    methods(Access='public')
        function Home(obj)
            % Move the UR simulation to the home configuration
            % TODO - confirm home position of UR3 and UR5
            joints = [...
                0.00;...
               -pi/2;...
                0.00;...
               -pi/2;...
                0.00;...
                0.00];
            obj.Joints = joints;
        end
        
        function Stow(obj)
            % Move the UR simulation to the stow configuration
            % TODO - confirm stow position of UR3 and UR5
            joints = [...
                0.00000;...
               -0.01626;...
               -2.77643;...
                1.22148;...
                1.57080;...
                0.00000];
            obj.Joints = joints;
        end
        
        function Zero(obj)
            % Move the UR simulation to the zero configuration
            obj.Joints = zeros(6,1);
        end
        
        function Undo(obj)
            % Undo the previous move of the UR
            alljoints = obj.Joints_Old;
            if ~isempty(alljoints)
                obj.Joints = alljoints(:,end);
                alljoints(:,end) = [];
                obj.Joints_Old = alljoints;
            end
        end
        
    end % end methods
    
    % --------------------------------------------------------------------
    % Private Methods
    % --------------------------------------------------------------------
    methods(Access='private')
        function tf = isQtRunning(obj)
            % Check if server is running
            str = sprintf('tasklist /fi "imagename eq %s"',obj.QtEXE);
            [~,cmdout] = system(str);
            % If the server is *not* running, we expect response resembling 
            % the following:
            %   "INFO: No tasks are running which match the specified
            %   criteria."
            tf = ~contains(cmdout,...
                'No tasks are running which match the specified criteria.');
        end
        
        function tf = existQt(obj)
            % Check if server exists
            str = fullfile(obj.QtPath,obj.QtEXE);
            tf = false;
            if exist(str,'file') == 2
                tf = true;
            end
        end
        
        function startQt(obj)
            % Start the server
            fstr = fullfile(obj.QtPath,obj.QtEXE);
            str = sprintf('start /min "" "%s" ',fstr);
            system(str);
        end
        
        function sendMsg(obj,varargin)
            % Send message to Qt server
            % obj.sendMsg(msg)
            msg = varargin{1};
            s = uint8(msg);
            write(obj.Client,s);
            % Wait for response
            while isempty(obj.Client.BytesAvailable)
                % Waiting for response from server
            end
        end
        
        function msg = receiveMsg(obj,varargin)
            % Receive message from Qt server
            % obj.receiveMsg(6,'double')
            dSize = varargin{1};
            dType = varargin{2};
            % TODO - check nargin, and variables
            msg = read(obj.Client,dSize,dType);
        end
        
        function task = pose2task(obj,pose)
            % Convert pose to task space
            H = pose;
            R = H(1:3,1:3);
            d = H(1:3,4);
            r = rotationMatrixToVector(R);
            task = [reshape(d,1,3),reshape(r,1,3)];
        end
        
        function pose = task2pose(obj,task)
            % Convert task space to pose
            d = task(1:3);
            r = task(4:6);
            R = rotationVectorToMatrix(r);
            H = eye(4);
            H(1:3,1:3) = R;
            H(1:3,4) = d;
            pose = H;
        end
    end
    
    % --------------------------------------------------------------------
    % Getters/Setters
    % --------------------------------------------------------------------
    methods
        % GetAccess & SetAccess ------------------------------------------
        
        % Joints - 1x6 array containing joint values (radians)
        function joints = get.Joints(obj)
            % Get current joint configuration of the simulation
            msg = 'getarmjoints';
            obj.sendMsg(msg);
            joints = obj.receiveMsg(6,'double');
            joints = reshape(joints,[],1);
        end
        
        function set.Joints(obj,joints)
            % Set the joint configuration of the simulation
            % movej([0,1.57,-1.57,3.14,-1.57,1.57],a=1.4, v=1.05, t=0, r=0)
            if numel(joints) ~= 6
                error('Joint vector must be specified as a 6-element array.');
            end
            
            msg = sprintf(...
                'movej([%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f])\n',...
                joints);
            obj.sendMsg(msg);
            rsp = obj.receiveMsg(1,'uint8');  
        end
        
        % JointI - individaul joints of the robot
        % Joint 1
        function joint = get.Joint1(obj)
            % Get current angle of joint 1
            joints = obj.Joints;
            joint = joints(1);
        end
        function set.Joint1(obj,joint)
            % Set current angle of joint 1
            joints = obj.Joints;
            joints(1) = joint;
            obj.Joints = joints;
        end
        % Joint 2
        function joint = get.Joint2(obj)
            % Get current angle of joint 2
            joints = obj.Joints;
            joint = joints(2);
        end
        function set.Joint2(obj,joint)
            % Set current angle of joint 2
            joints = obj.Joints;
            joints(2) = joint;
            obj.Joints = joints;
        end
        % Joint 3
        function joint = get.Joint3(obj)
            % Get current angle of joint 3
            joints = obj.Joints;
            joint = joints(3);
        end
        function set.Joint3(obj,joint)
            % Set current angle of joint 3
            joints = obj.Joints;
            joints(3) = joint;
            obj.Joints = joints;
        end
        % Joint 4
        function joint = get.Joint4(obj)
            % Get current angle of joint 4
            joints = obj.Joints;
            joint = joints(4);
        end
        function set.Joint4(obj,joint)
            % Set current angle of joint 4
            joints = obj.Joints;
            joints(4) = joint;
            obj.Joints = joints;
        end
        % Joint 5
        function joint = get.Joint5(obj)
            % Get current angle of joint 5
            joints = obj.Joints;
            joint = joints(5);
        end
        function set.Joint5(obj,joint)
            % Set current angle of joint 5
            joints = obj.Joints;
            joints(5) = joint;
            obj.Joints = joints;
        end
        % Joint 6
        function joint = get.Joint6(obj)
            % Get current angle of joint 6
            joints = obj.Joints;
            joint = joints(6);
        end
        function set.Joint6(obj,joint)
            % Set current angle of joint 6
            joints = obj.Joints;
            joints(6) = joint;
            obj.Joints = joints;
        end
        
        % Pose - 4x4 homogeneous transform representing the end-effector
        %        pose relative to the world frame
        function pose = get.Pose(obj)
            % Get the current end-effector pose of the simulation
            task = obj.Task;
            pose = obj.task2pose(task);
        end
        
        function set.Pose(obj,pose)
            % Set the current end-effector pose of the simulation
            task = pose2task(pose);
            obj.Task = task;
        end
        
        % Task - 1x6 array of [x,y,z,r1,r2,r3] matching the task variables 
        %        used by UR with the exception of linear units (x,y,z) that
        %        are specified in millimeters 
        function task = get.Task(obj)
            msg = 'getarmpose';
            obj.sendMsg(msg);
            task = obj.receiveMsg(6,'double');
            task(1:3) = task(1:3)*1000; % Convert from m to mm
            task = reshape(task,[],1);
        end
        
        function set.Task(obj,task)
            % movej([0,1.57,-1.57,3.14,-1.57,1.57],a=1.4, v=1.05, t=0, r=0)
            if numel(joints) ~= 6
                error('Task vector must be specified as a 6-element array.');
            end
            task(1:3) = task(1:3)./1000; % Convert from mm to m
            msg = sprintf(...
                'movej(p[%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f])\n',...
                task);
            obj.sendMsg(msg);
            rsp = obj.receiveMsg(1,'uint8');  
        end
        
        % ToolPose - 4x4 homogeneous transform representing the tool pose
        %            relative to the world frame
        function toolpose = get.ToolPose(obj)
            % Get the current tool pose of the simulation
            pose = obj.Pose;
            toolpose = pose * obj.FrameT;
        end
        
        function set.ToolPose(obj,toolpose)
            % Set the current tool pose of the simulation
            pose = toolpose * invSE(obj.FrameT);
            obj.Pose = pose;
        end
        
        % FrameT - Tool Frame (transformation relative to the End-effector
        %          Frame)
        function frameT = get.FrameT(obj)
            % Get the transformation relating the tool frame to the
            % end-effector frame
            frameT = obj.FrameT;
        end
        
        function set.FrameT(obj,frameT)
            % Set the transformation relating the tool frame to the
            % end-effector frame
            obj.FrameT = frameT;
        end
        
        function obj = set.Joints_Old(obj,allJoints)
            % Set the history for undo method
            n = 50; % Limit size of history
            % alljoints(:,end+1) = joints;
            if size(allJoints,2) > 50
                allJoints(:,1) = [];
            end
            obj.Joints_Old = allJoints;
        end
        
        % GetAccess ------------------------------------------------------
        
        % URmodel - Specified type of Universal Robot Manipulator
        function urmodel = get.URmodel(obj)
            urmodel = obj.URmodel;
        end
        % DHtable - DH table associated with robot
        function dhtable = get.DHtable(obj)
            q = obj.Joints;
            urMod = obj.URmodel;
            dhtable = UR_DHtable(urMod,q);
        end
 
    end % end methods
end % end classdef
