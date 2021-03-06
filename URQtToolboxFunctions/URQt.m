classdef URQt < matlab.mixin.SetGet % Handle
    % URQt handle class for creating a designated UR hardware connection.
    %
    %   obj = URQt creates a hardware connection object for a specific UR
    %
    % URQt Methods
    %   Initialize   - Initialize the URQt object.
    %   isMoving     - Determine whether the UR is currently moving.
    %   isGripMoving - Determine whether Robotiq gripper is moving.
    %   WaitForMove  - Wait until UR completes specified movement.
    %   WaitForGrip  - Wait until Robotiq gripper finishes movement.
    %   Home         - Move URQt to home joint configuration.
    %   Stow         - Move URQt to stow joint configuration.
    %   Zero         - Move URQt to zero joint configuration.
    %   *Undo        - Return URQt to previous joint configuration.
    %   get          - Query properties of the URQt object.
    %   set          - Update properties of the URQt object.
    %   delete       - Delete the URQt object and all attributes.
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
    %   *ToolPose    - 4x4 rigid body transform defining the tool pose
    %                 relative to the world frame (linear units are defined
    %                 in millimeters)
    %   *ToolTask    - 1x6 array of [x,y,z,r1,r2,r3] matching the task
    %                 variables used by UR with the exception of linear
    %                 units (x,y,z) that are specified in millimeters
    %
    % -Movement Parameters
    %   MoveType    % String describing move type (LinearTask or LinearJoint)
    %   JointAcc    % Joint acceleration of leading axis (rad/s^2)
    %   JointVel    % Joint speed of leading axis (rad/s)
    %   TaskAcc     % Task acceleration (mm/s^2)
    %   TaskVel     % Task speed (mm/s)
    %   BlendRadius % Blend radius between movements (mm)
    %   *MoveTime   % Movement time (s)
    %
    % -Frame Definitions
    %   *FrameT      - Tool Frame (transformation relative to the
    %                 End-effector Frame)
    %
    % -
    %
    % Example:
    %
    %       % Create and initialize hardware
    %       ur3e = URQt;              % Create URQt object
    %       ur3e.Initialize('UR3e');  % Designate as UR3e
    %
    %       % Send hardware to stow configuration
    %       ur3e.Stow;
    %       ur3e.WaitForMove;
    %       % Send hardware to home configuration
    %       ur3e.Home;
    %       ur3e.WaitForMove;
    %       % Send hardware to specified joint configuration
    %       ur3e.Joints = 2*pi*rand(1,6); % <--- BE CAREFUL!!!
    %       ur3e.WaitForMove;
    %
    % See also
    %
    %   M. Kutzer 26Mar2021, USNA
    
    % Updates
    %   26Aug2021 - Corrected get GripForce, converted GripForce and 
    %               GripSpeed to percentages
    
    % --------------------------------------------------------------------
    % General properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='private')
        IP          % String containing IP address for Qt connection
        Port        % Integer specifying port for Qt connection
        Client      % TCP client object for Qt connection
    end
    
    properties(GetAccess='public', SetAccess='public')
        Timeout     % Allowed time for TCP client to complete operations (s)
        ConnectTimeout   % Allowed time to connect to remove host (s)
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
        GripPosition    % scalar value for gripper position (mm)
        GripSpeed       % scalar value for gripper speed (TBD/s)
        GripForce       % scalar value for gripper force (TBD)
    end % end properties
    
    properties(GetAccess='public', SetAccess='public')
        MoveType    % String describing move type (LinearTask or LinearJoint)
        JointAcc    % Joint acceleration of leading axis (rad/s^2)
        JointVel    % Joint speed of leading axis (rad/s)
        TaskAcc     % Task acceleration (mm/s^2)
        TaskVel     % Task speed (mm/s)
        BlendRadius % Blend radius between movements (mm)
        %MoveTime   % Movement time (s)
    end
    properties(GetAccess='public', SetAccess='private')
        Jacobian    % Jacobian associated with robot
        DHtable     % DH table associated with robot
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
        function obj = URQt(varargin)
            % Create URQt Object
            
            % Define Qt executable path
            dName = 'urqt';
            fName = 'URQtSupport';
            obj.QtPath = fullfile(matlabroot,'toolbox',dName,fName);
            
            % Define Qt executable name
            obj.QtEXE = 'Roswell.exe';
            
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
            
            % Select robot
            % -> Define supported robot models
            URmods = {'UR3','UR3e','UR5','UR5e','UR10','UR10e'};
            if nargin < 1
                [s,v] = listdlg(...
                    'Name','Select Model',...
                    'PromptString','Select UR model:',...
                    'SelectionMode','single',...
                    'ListString',URmods);
                if v
                    % User selected model
                    obj.URmodel = URmods{s};
                else
                    % No value selected.
                    obj.URmodel = [];
                end
            else
                bin = strcmpi( URmods, varargin{1} );
                if nnz(bin) == 1
                    obj.URmodel = URmods{bin};
                else
                    obj.URmodel = [];
                end
            end
            
        end
        
        function delete(obj)
            % Object destructor
            
            % Shutdown arm
            obj.ShutdownArm;
            
            % Close the TCP/IP connection
            fprintf('Closing TCP Client: IP %s, Port %d...',...
                obj.IP,obj.Port);
            client = obj.Client;
            clear client
            obj.Client = [];
            fprintf('SUCCESS\n');
            
            % Close the Qt interface
            fprintf('Closing Qt interface...');
            tf = obj.stopQt;
            if tf
                fprintf('SUCCESS\n');
            else
                fprintf('FAILED\n');
            end
            
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
            
            % Clear old TCP client(s)
            if ~isempty( obj.Client )
                % Clear old TCP client(s)
                fprintf('Resetting TCP client...');
                client = obj.Client;
                obj.Client = [];
                clear client;
                fprintf('SUCCESS\n');
            else
                % No TCP client exists
            end
            
            % Initialize IP and Port
            % TODO - allow user to specify IP and Port
            obj.IP = '127.0.0.1';
            obj.Port = 8897;
            obj.Timeout = 5;
            obj.ConnectTimeout = 10;
            
            % Initialize parameters
            obj.MoveType = 'LinearJoint';
            obj.JointAcc = deg2rad(80); % rad/s^2
            obj.JointVel = deg2rad(60); % rad/s
            obj.TaskAcc  = 100.0;       % mm/s^2
            obj.TaskVel  = 100.0;       % mm/s
            obj.BlendRadius = 0;        % mm
            
            % TODO - specify and use terminator
            % TODO - specify terminator callback function
            fprintf('Initializing TCP Client: IP %s, Port %d...',...
                obj.IP,obj.Port);

            obj.Client = tcpclient(obj.IP,obj.Port,...
                'Timeout',obj.Timeout,...
                'ConnectTimeout',obj.ConnectTimeout);
            fprintf('SUCCESS\n');
            
            % Initialize arm
            obj.InitializeArm;
            
        end
    end % end methods
    
    % --------------------------------------------------------------------
    % General Use
    % --------------------------------------------------------------------
    methods(Access='public')
        function InitializeArm(obj)
            fprintf('Initializing arm...');
            msg = 'btnarm';
            obj.sendMsg(msg);
            out = obj.receiveMsg(1,'uint8');
            switch out
                case 2  % Arm is ready
                    fprintf('READY');
                case 1  % Arm is booting
                    fprintf('ARM BOOTING...');
                    
                    % Wait for arm to boot
                    t0 = tic;
                    tf = 30;
                    g = gifwait(0,'Initializing arm');
                    while obj.Client.BytesAvailable == 0
                        g = gifwait(g);
                        t = toc(t0);
                        if t >= tf
                            fprintf('This took longer than expected...');
                            break
                        end
                    end
                    delete(g.fig);
                    
                    out = obj.receiveMsg(1,'uint8');
                    if out == 2
                        fprintf('READY');
                    else
                        fprintf('UNKNOWN RESPONSE "%d"',out);
                    end
                otherwise
                    fprintf('UNKNOWN RESPONSE "%d"',out);
            end
            fprintf('\n');
        end
        
        function ShutdownArm(obj)
            %{
            fprintf('Shutting down arm...');
            msg = 'shutdown';
            %msg = 'power off';
            obj.sendMsg(msg);
            fprintf('COMMAND SENT\n');
            %}
            fprintf('ShutdownArm: This method is not fully implemented.\n');
        end
        
        function tf = isMoving(obj)
            % Check if UR is currently moving
            msg = 'getarmdone';
            obj.sendMsg(msg);
            out = obj.receiveMsg(1,'uint8');
            if out == 1
                % Motion is complete
                tf = false;
            else
                % Arm is moving
                tf = true;
            end
        end
        
        function tf = isGripMoving(obj)
            % Check if Robotiq is currently moving
            % TODO - This depends on a hard-coded pause that is clunky
            g0 = obj.GripPosition;
            pause(0.1);
            g1 = obj.GripPosition;
            if g0 == g1
                tf = false;
            else
                tf = true;
            end
        end
        
        function WaitForMove(obj)
            % Wait for arm to move
            fprintf('Waiting for UR move');
            while obj.isMoving
                fprintf('.');
            end
            fprintf('\n');
        end
        
        function WaitForGrip(obj)
            % Wait for gripper to move
            fprintf('Waiting for Robotiq move');
            while obj.isGripMoving
                fprintf('.');
            end
            fprintf('\n');
        end
        
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
                -1.570782,...
                -3.141604,...
                2.652899,...
                -4.223715,...
                0.000023,...
                4.712387];
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
        
        function tf = stopQt(obj)
            % Stop the server
            str = sprintf('taskkill /im "%s"',obj.QtEXE);
            [~,cmdout] = system(str);
            tf = contains(cmdout,'SUCCESS');
        end
        
        function sendMsg(obj,varargin)
            % Send message to Qt server
            % obj.sendMsg(msg)
            msg = varargin{1};
            s = uint8(msg);
            write(obj.Client,s);
            % Wait for response
            timeout = 10;
            t0 = tic;
            while isempty(obj.Client.BytesAvailable)
                % Waiting for response from server
                t = toc(t0);
                if t > timeout
                    warning('TIMEOUT - No response received from server');
                    break
                end
            end
        end
        
        function msg = receiveMsg(obj,varargin)
            % Receive message from Qt server
            % obj.receiveMsg(6,'double')
            dSize = varargin{1};
            dType = varargin{2};
            % TODO - check nargin, and variables
            try
                msg = read(obj.Client,dSize,dType);
            catch
                %warning('Timeout reached, no message received.');
                msg = nan;
            end
        end
        
        function task = pose2task(obj,pose)
            % Convert pose to task space
            H = pose;
            R = H(1:3,1:3);
            d = H(1:3,4);
            r = rotationMatrixToVector(R);
            task = [reshape(d,1,3),reshape(-r,1,3)];
        end
        
        function pose = task2pose(obj,task)
            % Convert task space to pose
            d = task(1:3);
            r = task(4:6);
            R = rotationVectorToMatrix(-r);
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
        % Timeout
        function set.Timeout(obj,timeout)
            switch lower( class(obj.Client) )
                case 'tcpclient'
                    obj.Client.Timeout = timeout;
                    obj.Timeout = timeout;
                otherwise
                    obj.Timeout = timeout;
            end
        end
        
        % Timeout
        function set.ConnectTimeout(obj,timeout)
            switch lower( class(obj.Client) )
                case 'tcpclient'
                    obj.Client.ConnectTimeout = timeout;
                    obj.ConnectTimeout = timeout;
                otherwise
                    obj.ConnectTimeout = timeout;
            end
        end
        
        % Move Type
        function moveType = get.MoveType(obj)
            moveType = obj.MoveType;
        end
        
        function set.MoveType(obj,moveType)
            switch lower(moveType)
                case 'linearjoint'
                    moveType = 'LinearJoint';
                case 'lineartask'
                    moveType = 'LinearTask';
                case 'joint'
                    moveType = 'LinearJoint';
                case 'task'
                    moveType = 'LinearTask';
                otherwise
                    error('Unrecognized MoveType "%s"',moveType);
            end
            obj.MoveType = moveType;
        end
        
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
            
            % Check specified joint array
            if numel(joints) ~= 6
                error('Joint vector must be specified as a 6-element array.');
            end
            
            jmsg = sprintf('[%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f]',joints);
            switch obj.MoveType
                case 'LinearJoint'
                    a = obj.JointAcc;           % rad/s^2
                    v = obj.JointVel;           % rad/s^2
                    r = obj.BlendRadius/1000;   % m
                    msg = sprintf('movej(%s,a=%9.6f, v=%9.6f, t=0, r=%9.6f)\n',...
                        jmsg,a,v,r);
                case 'LinearTask'
                    a = obj.TaskAcc/1000;       % m/s^2
                    v = obj.TaskVel/1000;       % m/s
                    r = obj.BlendRadius/1000;   % m
                    msg = sprintf('movel(%s,a=%9.6f, v=%9.6f, t=0, r=%9.6f)\n',...
                        jmsg,a,v,r);
                otherwise
                    error('Unrecognized move type "%s"',obj.MoveType);
            end
            
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
            task = obj.pose2task(pose);
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
            if numel(task) ~= 6
                error('Task vector must be specified as a 6-element array.');
            end
            
            task(1:3) = task(1:3)./1000; % Convert from mm to m
            tmsg = sprintf('[%9.6f,%9.6f,%9.6f,%9.6f,%9.6f,%9.6f]',task);
            switch obj.MoveType
                case 'LinearJoint'
                    a = obj.JointAcc;           % rad/s^2
                    v = obj.JointVel;           % rad/s^2
                    r = obj.BlendRadius/1000;   % m
                    msg = sprintf('movej(p%s,a=%9.6f, v=%9.6f, t=0, r=%9.6f)\n',...
                        tmsg,a,v,r);
                case 'LinearTask'
                    a = obj.TaskAcc/1000;       % m/s^2
                    v = obj.TaskVel/1000;       % m/s
                    r = obj.BlendRadius/1000;   % m
                    msg = sprintf('movel(p%s,a=%9.6f, v=%9.6f, t=0, r=%9.6f)\n',...
                        tmsg,a,v,r);
                otherwise
                    error('Unrecognized move type "%s"',obj.MoveType);
            end
            
            obj.sendMsg(msg);
            rsp = obj.receiveMsg(1,'uint8');
        end
        % Grip Position
        function gPos = get.GripPosition(obj)
            msg = 'get pos\n';
            obj.sendMsg(msg);
            gPos_uint8 = obj.receiveMsg(1,'double');
            gPos = gPos_uint8 * (52/255);
        end
        
        function set.GripPosition(obj,gPos)
            if gPos < 0
                warning('Gripper position must be between 0 and 52 mm');
                gPos = 0;
            end
            if gPos > 52
                warning('Gripper position must be between 0 and 52 mm');
                gPos = 52;
            end
            gPos_uint8 = uint8( gPos*(255/52) );
            msg = sprintf('set pos %d\n',gPos_uint8);
            obj.sendMsg(msg);
        end
        
        % Grip Speed
        function gSpeed = get.GripSpeed(obj)
            % TODO - figure out units and conversion for grip speed
            msg = 'get spe\n';
            obj.sendMsg(msg);
            gSpeed = obj.receiveMsg(1,'double');
            
            gSpeed = (gSpeed./255)*100; % Convert to percent
        end
        
        function set.GripSpeed(obj,gSpeed)
            % TODO - figure out what the actual unit conversion is and
            % limits are!
            % -> 0 - 255
            gSpeed = gSpeed/100; % convert to decimal
            if gSpeed > 1
                warning('Gripper speed cannot exceed 100%');
                gSpeed = 1;
            end
            if gSpeed < 0
                warning('Gripper speed cannot be below 0%');
                gSpeed = 0;
            end
            gSpeed = gSpeed*255;
            
            msg = sprintf('set spe %d\n',uint8(gSpeed));
            obj.sendMsg(msg);
        end
        
        % Grip Force
        function gForce = get.GripForce(obj)
            % TODO - figure out units and conversion for grip force
            msg = sprintf('get for\n');
            obj.sendMsg(msg);
            gForce = obj.receiveMsg(1,'double');
            
            gForce = (gForce./255)*100; % Convert to percent
        end
        
        function set.GripForce(obj,gForce)
            % TODO - figure out units and conversion for grip force
            gForce = gForce/100; % convert to decimal
            if gForce > 1
                warning('Gripper force cannot exceed 100%');
                gForce = 1;
            end
            if gForce < 0
                warning('Gripper force cannot be below 0%');
                gForce = 0;
            end
            gForce = gForce*255;
            
            msg = sprintf('set for %d\n',uint8(gForce));
            obj.sendMsg(msg);
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
        
        function set.Joints_Old(obj,allJoints)
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
        % TODO - allow user to calibrate robot to correct DH table
        function dhtable = get.DHtable(obj)
            q = obj.Joints;
            urMod = obj.URmodel;
            dhtable = UR_DHtable(urMod,q);
        end
        
        % Jacobian - Jacobian associated with robot
        function J = get.Jacobian(obj)
            q = obj.Joints;
            urMod = obj.URmodel;
            J = UR_Jacobian(urMod,q);
        end
    end % end methods
end % end classdef
