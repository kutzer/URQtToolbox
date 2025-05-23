classdef URQt < matlab.mixin.SetGet % Handle
    % URQt handle class for creating a designated UR hardware connection.
    %
    %   obj = URQt creates a hardware connection object for a specific UR
    %
    % Methods/Properties Note:
    %   - not all methods/properties are documented below.
    %   * indicates methods/properties still in development.
    %
    % Known Issue(s)
    % (1) Overwriting an existing URQt object calls the object destructor
    %     after the object constructor resulting in a closed IP connection
    %    BAD: Creating and overwriting a URQt object:
    %       ur = URQt('UR3e');
    %       ur = URQt('UR3e');
    %     -> This results in an object with a closed IP connection!
    %   GOOD: To correctly reinitialize an object, use:
    %       ur = URQt('UR3e');
    %       clear ur
    %       ur = URQt('UR3e');
    %
    % Example:
    %       % Create and initialize hardware (WaitOn = true [DEFAULT]) ---
    %       ur3e = URQt('UR3e');  % Create URQt object & designate UR3e
    %       ur3e.Initialize;      % Initialize object
    %
    %       % Send hardware to stow configuration
    %       ur3e.Stow;
    %       % Send hardware to home configuration
    %       ur3e.Home;
    %       % Send hardware to specified joint configuration
    %       ur3e.Joints = 2*pi*rand(1,6); % <--- BE CAREFUL!!!
    %       % ------------------------------------------------------------
    %
    %       % Create and initialize hardware (WaitOn = false) ------------
    %       ur3e = URQt('UR3e');  % Create URQt object & designate UR3e
    %       ur3e.Initialize;      % Initialize object
    %       ur3e.WaitOn = false;  % Turn off automatic wait
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
    %       % ------------------------------------------------------------
    %
    % See also
    %   URQtToolboxUpdate URQtToolboxVer UR_*
    %
    %   M. Kutzer, 26Mar2021, USNA

    % Updates
    %   26Aug2021 - Corrected get GripForce, converted GripForce and
    %               GripSpeed to percentages
    %   30Aug2021 - WaitOn functionality
    %   31Aug2021 - Added JointAcc, JointVel, TaskAcc, and TaskVel limits
    %               to set functions
    %   06Oct2021 - Updated if statements for JointVel, TaskVel, and
    %               TaskAcc to match true peak values
    %   06Oct2021 - Updated to add 2-second pause in Initialize
    %   17Feb2022 - Added ServoJ and SpeedJ methods
    %   24Feb2022 - Added FlushBuffer method
    %   03Mar2022 - Fixed acceleration limit
    %   03Mar2022 - Major revision to documentation
    %   03Mar2022 - Updated appendJointHistory usage
    %   13Feb2024 - Typo correction in joint velocity limits
    %   30Jan2025 - Updated to include arm, gripper, and dashboard commands
    %   03Apr2025 - Added tfSkipChecks to WaitForGrip

    % --------------------------------------------------------------------
    % General properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='private')
        IP          % String containing IP address for Qt connection
        Port        % Integer specifying port for Qt connection
        Client      % TCP client object for Qt connection
        isTCPmsg    % New TCP message flag
        TCPmsg      % TCP message
    end

    properties(GetAccess='public', SetAccess='public')
        Timeout         % Allowed time for TCP client to complete operations (s)
        ConnectTimeout  % Allowed time to connect to remove host (s)
    end

    properties(GetAccess='public', SetAccess='private')
        URmodel     % Specified type of Universal Robot Manipulator
    end

    properties(GetAccess='public', SetAccess='public')
        Joints      % 6x1 array containing joint values (radians)
        Pose        % 4x4 homogeneous transform representing the end-effector pose relative to the world frame
        Task        % 6x1 array containing end-effector pose in task space
        ToolPose    % 4x4 homogeneous transform representing the tool pose relative to the world frame
        ToolTask    % 6x1 array containing tool pose in task space
        GripPosition    % scalar value for gripper position (mm \in [0,52])
        GripSpeed       % scalar value for gripper speed (percent \in [0,100])
        GripForce       % scalar value for gripper force (percent \in [0,100])
    end

    properties(GetAccess='public', SetAccess='public')
        WaitOn      % Binary describing whether to automatically call WaitForMove and WaitForGrip
        MoveType    % String describing move type (LinearTask or LinearJoint)
        JointAcc    % Joint acceleration of leading axis (rad/s^2)
        JointVel    % Joint speed of leading axis (rad/s)
        TaskAcc     % Task acceleration (mm/s^2)
        TaskVel     % Task speed (mm/s)
        BlendRadius % Blend radius between movements (mm)
        Gain          % Used with ServoJ method
        BlockingTime  % Used with ServoJ & SpeedJ methods
        LookAheadTime % Used with ServoJ method
        %MoveTime   % Movement time (s)
    end

    properties(GetAccess='public', SetAccess='private')
        Jacobian_o % Base frame referenced Jacobian for current joint configuration [JTran; JRot]
        Jacobian_e % End-effector referenced Jacobian for current joint configuration [JTran; JRot]
        Jacobian_t % Tool frame referenced Jacobian for current joint configuration [JTran; JRot]
        Fkin_e     % Forward kinematics for current joint configuration (end-effector relative to base)
        Fkin_t     % Forward kinematics for current joint configuration (tool frame relative to base)
        DHtable    % DH table for current joint configuration (end-effector relative to base frame)
    end % end properties

    properties(GetAccess='public', SetAccess='private', Hidden=true)
        fcnJacobian_o % Anonymous base frame referenced Jacobian function [JTran; JRot]
        fcnJacobian_e % Anonymous end-effector referenced Jacobian function [JTran; JRot]
        fcnJacobian_t % Anonymous tool frame referenced Jacobian function [JTran; JRot]
        fcnFkin_e     % Anonymous forward kinematics function (end-effector relative to base frame)
        fcnDHtable    % Anonymous DH table function  (end-effector relative to base frame)
        symJoints     % Symbolic joint configuration (used for defining/updating anonymous functions)
        symFkin_e     % Symbolic forward kinematics (end-effector relative to base frame)
    end % end properties

    properties(GetAccess='public', SetAccess='public')
        FrameT      % Tool Frame (transformation relative to the End-effector Frame)
    end % end properties

    properties(GetAccess='public', SetAccess='private')
        JointPositionLimits % Joint position limits (radians)
        JointVelocityLimits % Joint velocity limits (radians/sec)
        JointAccelerationLimits % Joint acceleration limits (radians/sec^2)
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
        QtPath        % Path of Qt executable
        QtEXE         % Name of Qt executable
        JointHistory  % Previous joint configuration (used with Undo)
        GripHistory   % Previous grip position (used for isGripMoving)
        tfSkipChecks  % Logical value to skip receive checks
    end

    % --------------------------------------------------------------------
    % Debug properties
    % --------------------------------------------------------------------
    properties(GetAccess='public', SetAccess='public', Hidden=true)
        QtDebug     % Debug flag to surpress Rosette launch
    end

    % --------------------------------------------------------------------
    % Constructor/Destructor
    % --------------------------------------------------------------------
    methods(Access='public')

        % Create Object --------------------------------------------------
        function obj = URQt(varargin)
            % URQt creates the URQt object and initializes the Qt Executable
            % to the e-Series controller (Rosette vX.X)
            %   obj = URQt prompts the user to select the e-Series model
            %
            %   obj = URQt(urMod) allows the user to specify the e-Series
            %   model using a character array
            %
            %   obj = URQt(urMod,debug)
            %
            %   Input(s)
            %       urMod - character array specifying e-Series model
            %           urMod = 'UR3e'
            %           urMod = 'UR5e'
            %           urMod = 'UR10e'
            %       debug - logical scalar specifying debug status
            %
            %   Output(s)
            %       obj - URQt object
            %
            %   M. Kutzer, 26Mar2021, USNA

            % Apply debug flag
            %   ENABLE DEBUG: obj = URQt('UR3e',true)
            if nargin > 1
                fprintf(2,'!!! DEBUG FLAG DETECTED !!!\n');
                if islogical(varargin{2}) && isscalar(varargin{2})
                    obj.QtDebug = varargin{2};

                    if obj.QtDebug
                        fprintf(2,'DEBUG ENABLED: *.QtDebug = true;\n');
                    else
                        fprintf(2,'DEBUG DISABLED: *.QtDebug = false;\n');
                    end
                else
                    obj.QtDebug = false;
                    fprintf(2,'Non scalar or non logical debug flag specified.\n');
                    fprintf(2,'\tExample: "obj = URQt(''UR3e'',true)" to call debug\n')
                    fprintf(2,'DEBUG DISABLED: *.QtDebug = false;\n');
                end
            else
                obj.QtDebug = false;
            end

            % Define Qt executable path
            dName = 'urqt';
            fName = 'URQtSupport';
            if ~obj.QtDebug
                % Normal operation
                obj.QtPath = fullfile(matlabroot,'toolbox',dName,fName);
            else
                % Debug
                obj.QtPath = uigetdir(userpath,'Select Roswell "release" directory');
            end

            % Define Qt executable name
            obj.QtEXE = 'Roswell.exe';

            % Kill existing instances of Qt
            if obj.isQtRunning
                obj.killQt;
            end

            % Initialize Qt Executable
            obj.InitializeQt;

            % Select robot
            % -> Define supported robot models
            %URmods = {'UR3','UR3e','UR5','UR5e','UR10','UR10e'};
            URmods = {'UR3e','UR5e','UR10e'};
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

            while isempty( obj.URmodel )
                warning('Unacceptable UR Model specified.');
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
            end
        end
        % ----------------------------------------------------------------

        % Delete Object --------------------------------------------------
        function delete(obj)
            % DELETE delete the URQt object (object destructor)

            % Shutdown arm
            obj.ShutdownArm;

            % Close the TCP/IP connection
            obj.CloseTCP;

            % Close the Qt Executable
            tf = obj.stopQt;
            if ~tf
                obj.killQt;
            end

        end
        % ----------------------------------------------------------------

    end % end methods

    % --------------------------------------------------------------------
    % Initialization
    % --------------------------------------------------------------------
    methods(Access='public')
        function Initialize(obj,varargin)
            % INITIALIZE initializes the connection with the Qt executable,
            % and initializes the UR e-Series manipulator
            %   obj.Initialize
            %
            %   obj.Initialize(IP) specify the IP address, use default port
            %
            %   obj.Initialize([],Port) specify the port, use default IP
            %
            %   obj.Initialize(IP,Port) specify the IP address and port
            %
            %   Input(s)
            %       IP   - character array specifying IP address of the TCP
            %              Client connection to the Qt executable (default
            %              value '127.0.0.1')
            %       Port - integer value specifying the port of the TCP
            %              Client connection to the Qt executable (default
            %              value 8897)

            % Set default value(s)
            IP_val = '127.0.0.1';
            Port_val = 8897;
            IP_in = [];
            Port_in = [];

            % Parse Input(s)
            if numel(varargin) >= 2
                IP_in = varargin{1};
            end

            if numel(varargin) >= 3
                Port_in = varargin{2};
            end

            % TODO - Validate IP address and port

            % Change default(s)
            if ~isempty(IP_in)
                fprintf('Switching IP address from %s to %s\n',IP_val,IP_in);
                IP_val = IP_in;
            end

            if ~isempty(Port_in)
                fprintf('Switching Port from %d to %d\n',Port_val,Port_in);
                Port_val = Port_in;
            end

            % Initialize TCP/IP Client settings
            obj.IP = IP_val;
            obj.Port = Port_val;
            obj.Timeout = 5;
            obj.ConnectTimeout = 10;

            % Initialize TCP/IP
            obj.InitializeTCP;

            % Initialize parameters
            obj.WaitOn   = true;
            obj.MoveType = 'LinearJoint';
            obj.JointAcc = deg2rad(80); % rad/s^2
            obj.JointVel = deg2rad(60); % rad/s
            obj.TaskAcc  = 100.0;       % mm/s^2
            obj.TaskVel  = 100.0;       % mm/s
            obj.BlendRadius = 0;        % mm
            obj.Gain = 100;             % proportional gain, Used with ServoJ method
            obj.BlockingTime = 0.05;    % s, Used with ServoJ & SpeedJ methods
            obj.LookAheadTime = 0.1;    % s, Used with ServoJ method
            [q_lims,dq_lims,ddq_lims] = UR_jlims(obj.URmodel);
            obj.JointPositionLimits = q_lims;
            obj.JointVelocityLimits = dq_lims;
            obj.JointAccelerationLimits = ddq_lims;

            % -> Forward Kinematics/DH anonymous functions
            obj.fcnFkin_e = @(q)UR_fkin(obj.URmodel,q);
            obj.fcnDHtable = @(q)UR_DHtable(obj.URmodel,q);
            % -> Symbolic joint configuration
            syms q1 q2 q3 q4 q5 q6
            obj.symJoints = [q1; q2; q3; q4; q5; q6];
            % -> Symbolic forward kinematics
            obj.symFkin_e = obj.fcnFkin_e(obj.symJoints);
            % -> Jacobians
            obj.fcnJacobian_o = ...
                calculateJacobian(obj.symJoints,obj.symFkin_e,false,...
                'Reference','World','Order','TranslationRotation');
            obj.fcnJacobian_e = ...
                calculateJacobian(obj.symJoints,obj.symFkin_e,false,...
                'Reference','Body','Order','TranslationRotation');
            % -> Tool Frame offset
            %    NOTE: obj.fcnJacobian_t is set with FrameT (bad practice)
            obj.FrameT = eye(4);

            % Initialize arm
            obj.InitializeArm;

            % Initialize gripper
            obj.InitializeGripper;

        end

        function InitializeQt(obj)
            % INITIALIZEQT initializes the Qt executable

            fprintf('Starting Qt Executable...');
            if ~obj.isQtRunning
                if ~obj.existQt
                    fprintf('"%s" not found, FAILED\n',obj.QtEXE);
                    return
                end
                % Start Qt Executable
                obj.startQt;
                % Check if Qt Executable is running
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

                % Wait for QtEXE to start fully
                fprintf('Waiting for Qt Executable...');
                t0 = tic;
                tf = 2;
                g = gifwait(0,'Waiting for Qt Executable...');
                t = toc(t0);
                while t < tf
                    g = gifwait(g);
                    t = toc(t0);
                end
                delete(g.fig);
                fprintf('SUCCESS\n');

            else
                fprintf('SKIPPED\n');
                fprintf('\tQt Executable is already running.\n');
            end

        end

        function InitializeTCP(obj)
            % INITIALIZETCP initializes the TCP/IP connection with Rosette.

            % Clear old TCP client(s)
            if ~isempty( obj.Client )
                obj.CloseTCP;
            else
                % No TCP client exists
            end

            % Status update
            msg = 'Initializing TCP Client';
            fprintf('%s: IP %s, Port %04d...',...
                msg,obj.IP,obj.Port);

            % Define TCP client
            obj.Client = tcpclient(obj.IP,obj.Port,...
                'Timeout',obj.Timeout,...
                'ConnectTimeout',obj.ConnectTimeout);

            % Specify terminator
            configureTerminator(obj.Client,'LF');

            % Specify terminator-triggered callback
            obj.TCPmsg = '';
            obj.isTCPmsg = false;
            configureCallback(obj.Client,'terminator',...
                @obj.tcpTerminatorCallback);

            % TODO - specify terminator callback function
            fprintf('SUCCESS\n');
        end

        function InitializeArm(obj)
            % INITIALIZEARM initializes the UR e-Series and Robotiq Hand-E
            % gripper using the Qt command "btnarm".

            % TODO - Isolate "InitializeArm" from "InitializeGripper"
            %   *This will require changes to the Qt code*

            % Check for LOCAL mode
            [tfRmt,tfRsp] = obj.isRemote;
            if tfRsp && ~tfRmt
                % Arm is not in remote mode
                fprintf([...
                    'Controller is currently in LOCAL mode.\n\n',...
                    'Unable to automatically run POWER ON and BRAKE RELEASE.\n\n',...
                    ' -> Please proceed manually using the teach pendant.\n']);
                return
            end

            % Check if arm is already on
            if obj.isRunning
                % Arm is already initialized
                fprintf('Arm is already initialized and running.\n');
                return
            end

            % POWER_OFF -> IDLE -> RUNNING

            % Power on controller
            %   POWER_OFF -> IDLE
            if obj.isPowerOff
                obj.PowerOn;
            end

            tf = obj.CheckSafetyAndRecover;
            if tf
                % Release the brakes
                %   IDLE -> RUNNING
                if obj.isIdle
                    obj.BrakeRelease;
                else
                    fprintf('Robot failed to power on.\n');
                end
            else
                fprintf('Unable to initialize the arm.\n');
            end

        end

        function InitializeGripper(obj)
            % INITIALIZEGRIP initializes the Robotiq Hand-E gripper using
            % the Qt command 'btngrip'

            fprintf('Initializing gripper...');
            msg = 'btngrip';
            if obj.QtDebug
                fprintf('\n');
            end
            obj.sendMsg(msg);
            %out = obj.receiveMsg(1,'value')
            fprintf('SUCCESS\n')
        end

        function tcpTerminatorCallback(obj,src,event)
            % TCPTERMINATORCALLBACK provides a callback function to
            % automatically read lines from the MATLAB/Qt TCP connection
            % when a terminator is sent
            obj.isTCPmsg = true;
            obj.TCPmsg = char( readline(obj.Client) );

            % DEBUG
            if obj.QtDebug
                funcStr = 'tcpTerminatorCallback';
                fprintf('%30s: "%s"\n',funcStr,obj.TCPmsg);
            end
        end

    end % end methods

    % --------------------------------------------------------------------
    % General Use
    % --------------------------------------------------------------------
    methods(Access='public')
        function ShutdownArm(obj)
            % SHUTDOWNARM powers off the UR e-Series robot.
            %
            %   NOTE: This method is disabled because shutting down the arm
            %         occurs every time the destructor is run

            %{
            fprintf('Shutting down arm...');
            msg = 'shutdown';
            %msg = 'power off';
            obj.sendMsg(msg);
            fprintf('COMMAND SENT\n');
            %}
            fprintf('obj.ShutdownArm: This method is disabled.\n');
        end

        function tf = isMoving(obj)
            % ISMOVING checks to see if the UR e-Series manipulator is
            % currently moving
            %   tf = obj.isMoving
            %
            %   Output(s)
            %       tf - binary value describing whether the manipulator is
            %            moving
            %
            %           tf = true if the manipulator is moving,
            %           tf = false if the manipulator is not moving

            % Check if UR is currently moving
            msg = 'getarmdone';
            obj.sendMsg(msg);
            out = obj.receiveMsg(1,'value');
            if out == 1
                % Motion is complete
                tf = false;
            else
                % Arm is moving
                tf = true;
            end
        end

        function tf = isGripMoving(obj)
            % ISGRIPMOVING checks to see if the Robotiq Hand-E gripper is
            % currently moving
            %   tf = obj.isGripMoving
            %
            %   Output(s)
            %       tf - binary value describing whether the gripper is
            %            moving
            %
            %           tf = true if the gripper is moving,
            %           tf = false if the gripper is not moving

            % Check if Robotiq is currently moving
            % TODO - This depends on a hard-coded pause that is clunky
            
            % Pause duration
            dt = 0.1;
            % Prior grip position
            g0 = obj.GripHistory;
            % Pause
            pause(dt);
            % Debug
            if obj.QtDebug
                fprintf('pause(%.4f)\n',dt);
            end
            % Get current grip position
            g1 = obj.GripPosition;
            % Check grip position
            if g0 == g1
                tf = false;
            else
                tf = true;
            end
        end

        function WaitForMove(obj)
            % WAITFORMOVE blocks MATLAB execution until a manipulator
            % configuration is achieved

            % Wait for arm to move
            fprintf('Waiting for UR move');
            % -> Temporarily disable debug
            tmpQtDebug = obj.QtDebug;
            obj.QtDebug = false;
            % -> Wait for move
            while obj.isMoving
                fprintf('.');
            end
            fprintf('\n');
            % -> Restore debug state
            obj.QtDebug = tmpQtDebug;

            % Add joint configuration to the joint history
            obj.addJointHistory;
        end

        function WaitForGrip(obj)
            % WAITFORGRIP blocks MATLAB execution until a gripper position
            % is achieved

            % Wait for gripper to move
            fprintf('Waiting for Robotiq move');
            % -> Temporarily disable debug
            tmpQtDebug = obj.QtDebug;
            obj.QtDebug = false;
            % -> Wait for move
            obj.tfSkipChecks = true;
            while obj.isGripMoving
                fprintf('.');
            end
            fprintf('\n');
            obj.tfSkipChecks = false;
            % -> Restore debug state
            obj.QtDebug = tmpQtDebug;
        end

        function Home(obj)
            % HOME moves the UR to the home configuration
            %
            %   NOTE: This method overrides a obj.WaitOn = 0 property value
            %         and imposes a *temporary* obj.WaitOn = 1

            % TODO - confirm home position of UR3 and UR5
            joints = [...
                0.00;...
                -pi/2;...
                0.00;...
                -pi/2;...
                0.00;...
                0.00];
            obj.Joints = joints;

            if ~obj.WaitOn
                obj.WaitForMove;
            end
        end

        function Stow(obj)
            % STOW moves the UR to the stow configuration
            %
            %   NOTE: This method overrides a obj.WaitOn = 0 property value
            %         and imposes a *temporary* obj.WaitOn = 1

            % TODO - confirm stow position of UR3 and UR5

            % Define "stow" joint configuration
            joints = [...
                -1.570782,...
                -3.141604,...
                2.652899,...
                -4.223715,...
                0.000023,...
                4.712387];

            % Set joint configuration
            obj.Joints = joints;

            % Wait for move
            if ~obj.WaitOn
                obj.WaitForMove;
            end
        end

        function Zero(obj)
            % ZERO Move the UR to the zero configuration
            %
            %   NOTE: This method overrides a obj.WaitOn = 0 property value
            %         and imposes a *temporary* obj.WaitOn = 1.

            % Define "zero" joint configuration
            joints = zeros(6,1);

            % Set joint configuration
            obj.Joints = joints;

            % Wait for move
            if ~obj.WaitOn
                obj.WaitForMove;
            end
        end

        function Undo(obj)
            % UNDO moves the UR to the previous joint configuration
            %
            %   NOTES:
            %   (1) This method overrides a obj.WaitOn = 0 property value
            %       and imposes a *temporary* obj.WaitOn = 1.
            %   (2) This method is only retrieves previous joint
            %       configurations achieved using the obj.WaitForMove
            %       method which is run when obj.WaitOn = 1;

            alljoints = obj.JointHistory;
            if ~isempty(alljoints)
                % Get current move type
                moveType = obj.MoveType;

                % Switch to linear joint movement
                obj.MoveType = 'LinearJoint';

                % Move to previous joint configuration
                obj.Joints = alljoints(:,end);

                % Wait for move
                if ~obj.WaitOn
                    obj.WaitForMove;
                end

                % Remove previous joint configuration
                alljoints(:,end) = [];

                % Update configuration history
                obj.JointHistory = alljoints;

                % Set move type to current move type
                obj.MoveType = moveType;
            end
        end

        function FlushBuffer(obj)
            % FLUSHBUFFER clears all existing contents from the TCP Client
            % Buffer.
            %
            %   NOTE: This method should be used if/when the robot appears
            %         to give "bad" joint, task, or pose information

            % Flush the  TCP Client buffer
            fprintf('Flushing TCP Client buffer.\n');
            fprintf('Reading all data from TCP Client...');
            msg = read(obj.Client);
            obj.isTCPmsg = false;
            obj.TCPmsg = '';
            fprintf('SUCCESS\n');
            disp(msg);
        end

        function tf = isQtRunning(obj)
            % ISQTRUNNING checks if the Qt executable is running using the
            % "tasklist" system command in Windows

            str = sprintf('tasklist /fi "imagename eq %s"',obj.QtEXE);
            [~,cmdout] = system(str);
            % If the server is *not* running, we expect response resembling
            % the following:
            %   "INFO: No tasks are running which match the specified
            %   criteria."
            tf = ~contains(cmdout,...
                'No tasks are running which match the specified criteria.');
        end

        function tf = isClientConnected(obj)
            % ISCLIENTCONNECTED checks if the TCP client is connected

            client = obj.Client;

            % Check for uninitialized client
            if isempty(client)
                tf = false;
                return
            end

            % Check if client is valid
            if ~isvalid(client)
                tf = false;
                return
            end

            % Check client type
            if ~isa(client, 'tcpclient')
                tf = false;
                return
            end

            % Check client status
            try
                % TODO - check other stuff
                [cIP,cPort] = InfoTCP(obj);
                tfOpen    = matches(lower(client.Status),'open');
                tfAddress = matches(cIP,obj.IP);
                tfPort    = cPort == obj.Port;
            catch
                tf = false;
                return
            end

            % Check parameters
            if tfOpen && tfAddress && tfPort
                tf = true;
            else
                tf = false;
            end
        end

        function tfKnownErr = CheckConnectionAndRecover(obj)
            % CHECKCONNECTIONANDRECOVER checks the Qt executable and the TCP/IP connection and attempts to recover is known errors are found
            %	tfKnownErr = obj.CheckConnectionAndRecover

            tfKnownErr = false;

            % Check Qt Executable
            if ~obj.isQtRunning
                fprintf(2,'\n!!! The Qt Executable is not running. Attempting to recover. !!!\n\n');
                obj.CloseTCP;
                obj.killQt;
                obj.InitializeQt;

                tfKnownErr = true;
            end

            % Check IP Client
            if ~obj.isClientConnected
                % Check if user has initialized port and IP
                if isempty(obj.Port) || isempty(obj.IP)
                    errMsg = sprintf([...
                        '\n',...
                        'The URQt object has not been initialized.\n',...
                        ' -> Please run *.Initialize to continue.']);
                    error(errMsg);
                else
                    fprintf(2,'\n!!! The TCP client is not connected. Attempting to recover. !!!.\n\n');
                end
                % Initialize IP connection
                obj.InitializeTCP;

                tfKnownErr = true;
            end
        end

        function varargout = CheckSafetyAndRecover(obj)
            % CHECKSAFETYANDRECOVER checks the safety status of the robot
            % and attempts to recover.
            
            %status = '';
            %chkMax = 15;
            %chk = 0;
            %while isempty(status)
            status = obj.SafetyStatus;
            %    drawnow;
            %    chk = chk+1;
            %    if chk >= chkMax
            %        break
            %    end
            %end

            tf = false;
            switch status
                case 'PROTECTIVE_STOP'
                    fprintf(2,'\n!!! The UR is in a safety state: %s. Attempting to recover. !!!\n\n',status);
                    obj.UnlockProtectiveStop;
                    tf = true;
                case 'NORMAL'
                    % System is normal, do nothing
                    tf = true;
                otherwise
                    msg = sprintf([...
                        '\n!!! The UR is in a safety state: %s. Unable to recover automatically. !!!\n',...
                        ' -> Use the teach pendant to clear the current safety state before continuing.\n\n'],...
                        status);
                    fprintf(2,'%s',msg);
            end

            if nargout > 0
                varargout{1} = tf;
            end
        end
    end

    % --------------------------------------------------------------------
    % UR Dashboard Server Methods
    % --------------------------------------------------------------------
    methods(Access='public')

        function [tf,tfRsp] = isRemote(obj)
            % ISREMOTE checks if the controller is in remote or local
            % control mode.
            %   [tf,tfRsp] = obj.isRemote;
            %
            %   Output(s)
            %          tf - logical scalar that is "true" if the robot is in
            %               remote mode
            %       tfRsp - logical scalar that is "true" if the robot
            %               responded to the command
            %
            %   M. Kutzer, 04Feb2025, USNA
            msg = obj.sendCmdDashboard('is in remote control');

            if isempty(msg)
                warning('Controller is not responding.');
                tf = false;
                tfRsp = false;
                return
            end

            switch lower(msg)
                case 'true'
                    tf = true;
                    tfRsp = true;
                case 'false'
                    tf = false;
                    tfRsp = true;
                otherwise
                    warning('Unexpected response.')
                    tf = false;
                    tfRsp = false;
            end
        end

        function rMode = RobotMode(obj)
            % ROBOTMODE gets the current mode of the UR controller.
            %   rMode = obj.RobotMode
            %
            %   Output(s)
            %       rMode - character array describing current robot mode
            %               [1]
            %           NO_CONTROLLER
            %           DISCONNECTED
            %           CONFIRM_SAFETY
            %           BOOTING
            %           POWER_OFF
            %           POWER_ON
            %           IDLE
            %           BACKDRIVE
            %           RUNNING
            %
            %   Reference(s)
            %       [1] https://s3-eu-west-1.amazonaws.com/ur-support-site/42728/DashboardServer_e-Series_2022.pdf

            persistent rMode_past

            msg = obj.sendCmdDashboard('robotmode');
            preamble = 'Robotmode: ';

            % No message
            if isempty(msg)
                warning('Controller is not responding.');
                rMode = '';
                rMode_past = rMode;
                return
            end

            % "true" Return
            if matches(msg,'true')
                % Use previous robot mode
                rMode = rMode_past;
                return
            end

            % Message is too short
            if numel(msg) <= numel(preamble)
                warning('Unexpected response from controller: "%s"',msg);
                rMode = '';
                rMode_past = rMode;
                return
            end

            % Message missing preamble
            if ~matches(msg(1:numel(preamble)),preamble)
                warning('Unexpected response from controller: "%s"',msg);
                rMode = '';
                rMode_past = rMode;
                return
            end

            idx0 = numel(preamble)+1;
            idxf = numel(msg);
            rMode = msg(idx0:idxf);

            % Capture previous rMode
            rMode_past = rMode;
        end

        function status = SafetyStatus(obj)
            % SAFETYSTATUS gets the current safety status of the UR
            % controller.
            %   status = obj.SafetyStatus
            %
            %   Output(s)
            %       status - character array describing current safety
            %                status [1]
            %           NORMAL
            %           REDUCED
            %           PROTECTIVE_STOP
            %           RECOVERY
            %           SAFEGUARD_STOP
            %           SYSTEM_ EMERGENCY_STOP
            %           ROBOT_ EMERGENCY_STOP
            %           VIOLATION
            %           FAULT
            %           AUTOMATIC_MODE_SAFEGUARD_STOP
            %           SYSTEM_THREE_POSITION_ENABLING_STOP
            %
            %   Reference(s)
            %       [1] https://s3-eu-west-1.amazonaws.com/ur-support-site/42728/DashboardServer_e-Series_2022.pdf

            msg = obj.sendCmdDashboard('safetystatus');
            preamble = 'Safetystatus: ';

            % No message
            if isempty(msg)
                warning('Controller is not responding.');
                status = '';
                %status_past = status;
                return
            end

            % "true" Return
            % if matches(msg,'true')
            %     % Use previous robot mode
            %     status = status_past;
            %     return
            % end

            % Message is too short
            if numel(msg) <= numel(preamble)
                warning('Unexpected response from controller: "%s"',msg);
                status = '';
                %status_past = status;
                return
            end

            % Message missing preamble
            if ~matches(msg(1:numel(preamble)),preamble)
                warning('Unexpected response from controller: "%s"',msg);
                status = '';
                %status_past = status;
                return
            end

            idx0 = numel(preamble)+1;
            idxf = numel(msg);
            status = msg(idx0:idxf);

            % Capture previous rMode
            %status_past = status;
        end

        function tf = isPowerOff(obj,rMode)
            % ISPOWEROFF returns true if the robot power is off.

            % Set default(s)
            if nargin < 2
                rMode = obj.RobotMode;
            end

            % Check state
            if matches(rMode,'POWER_OFF')
                tf = true;
            else
                tf = false;
            end
        end

        function tf = isPowerOn(obj,rMode)
            % ISPOWERON returns true if the robot power is on.
            % POWER_OFF -> POWER_ON -> BOOTING -> IDLE -> RUNNING

            % Set default(s)
            if nargin < 2
                rMode = obj.RobotMode;
            end

            % Check related states
            if obj.isRunning(rMode)
                tf = true;
                return;
            end
            if obj.isIdle(rMode)
                tf = true;
                return;
            end
            if obj.isBooting(rMode)
                tf = true;
                return
            end

            % Check for exact power-on state
            if matches(rMode,'POWER_ON')
                tf = true;
            else
                tf = false;
            end
        end

        function tf = isBooting(obj,rMode)
            % ISBOOTING returns true if the robot is booting.

            % Set default(s)
            if nargin < 2
                rMode = obj.RobotMode;
            end

            % Check state
            if matches(rMode,'BOOTING')
                tf = true;
            else
                tf = false;
            end
        end

        function tf = isIdle(obj,rMode)
            % ISIDLE returns true if the robot is idle.

            % Set default(s)
            if nargin < 2
                rMode = obj.RobotMode;
            end

            % Check state
            if matches(rMode,'IDLE')
                tf = true;
            else
                tf = false;
            end
        end

        function tf = isRunning(obj,rMode)
            % ISRUNNING returns true if the robot is running.

            % Set default(s)
            if nargin < 2
                rMode = obj.RobotMode;
            end

            % Check state
            if matches(rMode,'RUNNING')
                tf = true;
            else
                tf = false;
            end
        end

        function UnlockProtectiveStop(obj)
            % UNLOCKPROTECTIVESTOP removes a protective stop on the
            % manipulator.
            fprintf('Unlocking protective stop...');
            msg = obj.sendCmdDashboard('unlock protective stop');
            % TODO - check message
            fprintf('SUCCESS\n');
        end

        function PowerOn(obj)
            % POWERON powers the UR on

            msg = obj.sendCmdDashboard('power on');

            switch lower(msg)
                case 'powering on'
                    % Robot is powering on
                    % -> Temporarily disable debug
                    tmpQtDebug = obj.QtDebug;
                    obj.QtDebug = false;
                    % -> Wait for IDLE
                    tfSuccess = true;
                    fprintf('Waiting for robot to power on...');
                    t0 = tic;
                    tf = 45;
                    g = gifwait(0,'Waiting for robot to power on...');
                    while ~obj.isIdle
                        g = gifwait(g);
                        t = toc(t0);
                        if t > tf
                            tfSuccess = false;
                            break
                        end
                        pause(0.05);
                        % Debug
                        if obj.QtDebug
                            fprintf('pause(%.4f)\n',0.05);
                        end
                    end
                    delete(g.fig);
                    if tfSuccess
                        fprintf('SUCCESS\n');
                    else
                        fprintf('TIMEOUT\n');
                    end
                    % -> Restore debug state
                    obj.QtDebug = tmpQtDebug;

                otherwise
                    fprintf('Unexpected response: %s\n',msg);
            end
        end

        function BrakeRelease(obj)
            % BRAKERELEASE powers the UR on

            msg = obj.sendCmdDashboard('brake release');

            switch lower(msg)
                case 'brake releasing'
                    % Robot is powering on
                    % -> Temporarily disable debug
                    tmpQtDebug = obj.QtDebug;
                    obj.QtDebug = false;
                    % -> Wait for IDLE
                    tfSuccess = true;
                    fprintf('Waiting for robot to release brakes...');
                    t0 = tic;
                    tf = 45;
                    g = gifwait(0,'Waiting for robot to release brakes...');
                    while ~obj.isRunning
                        g = gifwait(g);
                        t = toc(t0);
                        if t > tf
                            tfSuccess = false;
                            break
                        end
                        pause(0.05);
                        % Debug
                        if obj.QtDebug
                            fprintf('pause(%.4f)\n',0.05);
                        end
                    end
                    delete(g.fig);
                    if tfSuccess
                        fprintf('SUCCESS\n');
                    else
                        fprintf('TIMEOUT\n');
                    end
                    % -> Restore debug state
                    obj.QtDebug = tmpQtDebug;

                otherwise
                    fprintf('Unexpected response: %s\n',msg);
            end
        end

    end % end methods

    % --------------------------------------------------------------------
    % URScript Direct Messaging
    % --------------------------------------------------------------------
    methods(Access='public')
        function msgOut = sendCmdArm(obj,msg)
            % SENDCMDARM sends a URScript "arm" command
            %   obj.sendCmdArm(msg)
            %
            %   Input(s)
            %       msg - 1xN character array defining arm command
            %
            %   M. Kutzer, 30Jan2025, USNA
            obj.sendMsg(sprintf('sendarmcmd%s',msg));
            msgOut = obj.receiveMsg(1,'double');
        end

        function msgOut = sendCmdGripper(obj,msg)
            % SENDCMDGRIPPER sends a URCap "gripper" command
            %   obj.sendCmdGripper(msg)
            %
            %   Input(s)
            %       msg - 1xN character array defining arm command
            %
            %   M. Kutzer, 30Jan2025, USNA
            obj.sendMsg(sprintf('sendgripcmd%s',msg));

            % Wait for response
            while obj.Client.BytesAvailable < 2
                % Wait ...
                drawnow
            end

            msgOut = '';
            waitForMsg = true;
            while waitForMsg
                msgTMP = obj.receiveMsg(obj.Client.BytesAvailable,'char');
                msgOut = [msgOut,msgTMP];
                if matches(msgOut(end),'$')
                    waitForMsg = false;
                end
            end
            msgOut = obj.parseCmdResponse( msgOut );
        end

        function msgOut = sendCmdDashboard(obj,msg)
            % SENDCMDDASHBOARD sends a URScript "dashboard" command
            %   obj.sendCmdDashboard(msg)
            %
            %   Input(s)
            %       msg - 1xN character array defining arm command
            %
            %   M. Kutzer, 30Jan2025, USNA
            obj.sendMsg(sprintf('senddashcmd%s',msg));
            msgOut = obj.tcpGetMessage;
            msgOut = obj.parseCmdResponse( msgOut );

            % DEBUG
            if obj.QtDebug
                funcStr = 'sendCmdDashboard';
                fprintf('%30s: "%s"\n',funcStr,msgOut);
            end
        end
    end

    % --------------------------------------------------------------------
    % Private URScript Command Response Method(s)
    % --------------------------------------------------------------------
    methods(Access='private')
        function out = parseCmdResponse(obj,msg)
            % PARSECMDRESPONSE parses a command response with $___$
            % character brackets.
            expression = '\$(.*?)\$'; % Non-greedy match between $ symbols
            tokens = regexp(msg, expression, 'tokens');

            if ~isempty(tokens)
                out = tokens{1}{1};
            else
                out = '';
            end

            if obj.QtDebug
                funcStr = 'parseCmdResponse';
                fprintf('%30s: "%s" (IN)\n',funcStr,msg);
                fprintf('%30s:  "%s" (OUT)\n',funcStr,out);
            end
        end

        function msg = tcpGetMessage(obj,dt)
            % TCPGETMESSAGE waits for a new line from the MATLAB/Qt TCP
            % connection, and returns the applicable message. If no
            % message, a warning is provided and msg = ''
            if nargin < 2
                dt = obj.Timeout;
            end
            
            if obj.tfSkipChecks
                dt = 0.1;
            end

            t0 = tic;
            while ~obj.isTCPmsg
                t = toc(t0);
                if t >= dt
                    if ~obj.tfSkipChecks
                        % Notify user that timeout was reached
                        fprintf(...
                            'tcpGetMessage: Timeout (%.4f s) reached.\n',dt);

                        % Check Qt & TCP/IP and try to recover
                        %obj.CheckConnectionAndRecover;

                        % Return empty message
                    end
                    msg = '';
                    return
                end
                drawnow
            end

            % Get message
            msg = obj.TCPmsg;
            % Reset message flag
            obj.isTCPmsg = false;
        end
    end

    % --------------------------------------------------------------------
    % URScript Programming Methods
    % --------------------------------------------------------------------
    methods(Access='public')
        function ServoJ(obj,q)
            % SERVOJ runs the URScript servoj command using a specified
            % joint configuration.
            %   obj.SeroJ(q)
            %
            %   Input(s)
            %       q - 6-element joint configuration in radians
            %
            %   Applicable properties:
            %       obj.Gain          - proportional gain for following
            %                           target position, range [100,2000]
            %       obj.BlockTime     - time (s) where the command is
            %                           controlling robot
            %       obj.LookAheadTime - time (s), range [0.03,0.2]
            %                           smoothens the trajectory with this
            %                           lookahead time
            %
            %   NOTE: Adjusting the following parameters may also impact
            %         performance
            %       (1) obj.JointAcc - maximum joint acceleration of
            %                          leading axis (rad/s^2)
            %       (2) obj.JointVel - maximum joint speed of leading axis
            %                          (rad/s)
            %
            %   M. Kutzer, 24Feb2022, USNA

            % TODO - check joint limits

            % Get necessary properties
            a = 0; % <-- Not used in current version
            v = 0; % <-- Not used in current version
            gain = obj.Gain;
            t = obj.BlockingTime;
            lookAheadTime = obj.LookAheadTime;

            msg = sprintf('servoj([%.4f,%.4f,%.4f,%.4f,%.4f,%.4f], %d, %d, %.3f, %.3f, %.3f)\n',...
                q(1),q(2),q(3),q(4),q(5),q(6),a,v,t,lookAheadTime,gain);

            obj.sendMsg(msg);
            %rsp = obj.receiveMsg(1,'value');
        end

        function SpeedJ(obj,dq)
            % SPEEDJ runs the URScript speedj command using a specified
            % joint velocity vector array
            %   obj.SpeedJ(dq)
            %
            %   Input(s)
            %       dq - 6-element joint velocity in radians/sec
            %
            %   Applicable properties:
            %       obj.JointAcc - maximum joint acceleration of
            %                      leading axis (rad/s^2)
            %       obj.BlockTime - time (s) where the command is
            %                       controlling robot
            %
            %   M. Kutzer, 24Feb2022, USNA

            % TODO - check velocity limits

            % Get necessary properties
            a = obj.JointAcc; % Joint acceleration of leading axis
            t = obj.BlockingTime;

            msg = sprintf('speedj([%.4f,%.4f,%.4f,%.4f,%.4f,%.4f], %.3f, %.3f)\n',...
                dq(1),dq(2),dq(3),dq(4),dq(5),dq(6),a,t);

            obj.sendMsg(msg);
            rsp = obj.receiveMsg(1,'value');
        end
    end

    % --------------------------------------------------------------------
    % Private Methods
    % --------------------------------------------------------------------
    methods(Access='private')
    %methods(Access='public') % DEBUG

        function CloseTCP(obj)
            % CLOSETCP closes the TCP/IP connection with Rosette.
            msg = 'Closing TCP Client';
            [cIP,cPort] = obj.InfoTCP;
            fprintf('%s: IP %s, Port %04d...',msg,cIP,cPort);
            client = obj.Client;
            obj.Client = [];
            clear client
            fprintf('SUCCESS\n');

            % Check for additional TCP clients
            clients = findTcpClients();

            if ~isempty(clients)
                error('Oh crap!');
            end
        end

        function [IP,Port] = InfoTCP(obj)
            % INFOTCP gets the TCP client IP and port info
            IP = '[NOT CONNECTED]';
            Port = 0;
            client = obj.Client;
            try
                IP = client.Address;
                Port = client.Port;
            catch
                % Ignore error(s)
            end
        end

        function tf = existQt(obj)
            % EXISTQT checks to see if the Qt executable exists in the
            % MATLAB path
            str = fullfile(obj.QtPath,obj.QtEXE);
            tf = false;
            if exist(str,'file') == 2
                tf = true;
            end
        end

        function startQt(obj)
            % START Start the Qt executable using the "start" system
            % command in Windows
            fstr = fullfile(obj.QtPath,obj.QtEXE);
            str = sprintf('start /min "" "%s" ',fstr);
            system(str);
        end

        function tf = stopQt(obj)
            % STOPQT "stop" instance(s) of the Qt executable using the
            % "taskkill /im" system command in Windows
            fprintf('Stopping existing instance of "%s"...',obj.QtEXE);
            try
                str = sprintf('taskkill /im "%s"',obj.QtEXE);
                [~,cmdout] = system(str);
                tf = contains(cmdout,'SUCCESS');

                if tf
                    fprintf('SUCCESS\n');
                else
                    fprintf('FAILED\n');
                end
            catch
                tf = false;
                fprintf('FAILED\n');
            end
        end

        function tf = killQt(obj)
            % KILLQT "kills" instance(s) of the Qt executable using the
            % "taskkill /IM ... /F" system command in Windows
            fprintf('Killing existing instance of "%s"...',obj.QtEXE);
            try
                str = sprintf('taskkill /IM "%s" /F',obj.QtEXE);
                [~,cmdout] = system(str);
                %sprintf('%s/n',cmdout);
                tf = true;
                fprintf('SUCCESS\n');
            catch
                tf = false;
                fprintf('FAILED\n');
            end
        end

        function sendMsg(obj,varargin)
            % SENDMSG writes a specified TCP Client message to the TCP
            % Server running in the Qt executable
            %   obj.sendMsg(msg)
            %
            %   Input(s)
            %       msg - 1xN character array specifying the desired
            %             message

            msg = varargin{1};

            % DEBUG
            if obj.QtDebug
                funcStr = 'sendMsg';
                % Replace LF/CR in debug display
                msgStr = msg;
                msgStr = strrep(msgStr, char(13), '[CR]'); % Carriage Return (CR)
                msgStr = strrep(msgStr, char(10), '[LF]'); % Line Feed (LF)
                % Display
                fprintf('%30s: "%s"\n',funcStr,msgStr);
            end
            
            % Convert message to unsigned 8-bit integer
            s = uint8(msg);
            % Reset message flag
            obj.isTCPmsg = false;

            try
                % Write message
                write(obj.Client,s);
            catch ME
                % Check Qt & TCP/IP and try to recover
                tfKnownErr = obj.CheckConnectionAndRecover;

                if ~tfKnownErr
                    % Throw error
                    rethrow(ME);
                end
            end
            
        end
        
        function waitForMsg(obj,timeout)
            % WAITFORMSG waits for the buffer to contain a non-zero value
            % or for the isTCPmsg flag to be true.
            %   waitForMsg(timeout)
            %
            %   Input(s)
            %       timeout - [OPTIONAL] positive scalar defining timeout 
            %                 in seconds. Default value is 10.
            %
            %   Output(s)
            %       [NONE]
            %
            %   M. Kutzer, 27Mar2025
            
            % Wait for response
            if nargin < 2
                timeout = 10;
            end

            t0 = tic;
            while isempty(obj.Client.BytesAvailable) && ~obj.isTCPmsg
                % Waiting for response from server
                t = toc(t0);
                if t > timeout
                    warning('waitForMsg: Timeout (%.4f s) reached.',timeout);
                    break
                end
            end

        end
        function msg = receiveMsg(obj,varargin)
            % RECEIVEMSG receives a message message from the TCP client
            % connected to the TCP Server running in the Qt executable
            %   receiveMsg(dataSize,dataType)
            %
            %   Input(s)
            %       dataSize - positive integer value specifying the number
            %                  values associated with the message
            %       dataType - 1xN character array specifying the type of
            %                  data expected
            %
            %   Example(s)
            %       % Read six double precision floating point values
            %       obj.receiveMsg(6,'array')
            %
            %   NOTE(s)
            %   (1) The waring recommendation does not include checking
            %       that the Rosette Vx.x network settings match that of
            %       the robot. To do this:
            %       (a) Confirm that Rosette Vx.x has the same static IP as
            %           the robot:
            %           Rosette Vx.x
            %               Network
            %                   Net Address
            %                       Robot Net URL
            %       (b) Confirm that the static IP settings associated with
            %           the network card connected to the robot are correct
            %           Example:
            %               Robot
            %                   IP Address: 10.0.0.2
            %                   Subnet Mask: 255.255.255.0
            %                   Default Gateway: 10.0.0.1
            %                   Preferred/Alternative DNS Server: 8.8.8.8
            %               Network Card
            %                   IP Address: 10.0.0.1
            %                   Subnet Mask: 255.255.255.0
            %                   Default Gateway: 10.0.0.2
            %                   Preferred/Alternative DNS Server: 8.8.8.8

            % TODO - check nargin, and variables

            % Parse inputs
            dSize = varargin{1};
            dType = varargin{2};
            
            % Wait for message
            obj.waitForMsg;

            % Receive message
            try
                msgRCV = obj.tcpGetMessage;
            catch ME
                if ~obj.isQtRunning
                    fprintf('The Qt Executable is not running. Attempting to recover.\n');
                    % TODO - Write recovery 
                else
                    % Throw error
                    rethrow(ME);
                end
            end

            % DEBUG
            if obj.QtDebug
                funcStr = 'receiveMsg';
                fprintf('%30s: "%s"\n',funcStr,msgRCV);
            end

            switch lower(dType)
                case 'char'
                    msg = msgRCV;
                case 'string'
                    msg = string(msgRCV);
                case 'value'
                    msg = str2double(msgRCV);
                case 'array'
                    % -> Array of bytes defined as comma separated
                    %    digits

                    try
                        % Check for comma
                        if ~matches(msgRCV(end),',')
                            warning('Possible array message issue.');
                        end

                        msgRCV = sprintf('[%s]',msgRCV(1:end-1));
                        vec = eval(msgRCV);
                        N = numel(vec);
                        if (N/dSize) ~= round(N/dSize)
                            warning('Recieved message is the wrong length.');
                        end
                        vecReshape = uint8( reshape(vec,[],dSize).' );

                        msg = nan(size(vecReshape,1),1);
                        for i = 1:size(vecReshape,1)
                            msg(i) = typecast(vecReshape(i,:),'double');
                        end
                    catch ME
                        warning('Unexpected message.');
                        msg = nan(1,dSize);
                    end

                case 'double'
                    % -> Array of bytes defined as characters
                    %    !!! KNOWN ISSUE !!!
                    %    -> Using this message format can conflict
                    %       with the TCP terminator

                    % Parse message
                    N = numel(msgRCV);
                    if (N/dSize) ~= round(N/dSize)
                        warning('Recieved message is the wrong length.');
                    end
                    msgReshape = uint8( reshape(msgRCV,[],dSize).' );

                    for i = 1:size(msgReshape,1)
                        msg(i) = typecast(msgReshape(i,:),dType);
                    end

                otherwise
                    warning('Unexpected dType = "%s"',dType);
                    msg = msgRCV;
            end
        end

        function task = pose2task(obj,pose)
            % POSE2TASK converts a pose (element of SE(3)) to its
            % corresponding task space representation [k; d]
            %   task = obj.pose2task(pose)
            %
            %   Input(s)
            %       pose - 4x4 array defining the end-effector pose
            %              relative to the base frame (must be a valid
            %              element of SE(3))
            %
            %   Output(s)
            %       task - 6x1 array defining a task configuration
            %
            %       task = [d] (3-element translation parameterization)
            %              [k] (3-element rotation parameterization)

            % TODO - check for valid element of SE(3)

            % Define pose
            H_e2o = pose;

            % Isolate rotation and translation
            R_e2o = H_e2o(1:3,1:3);
            d_e2o = H_e2o(1:3,4);

            % Map rotation to rotation parameterization
            k_e2o = -rotationMatrixToVector(R_e2o);

            % Package task configuration
            task = [reshape(d_e2o,1,3),reshape(k_e2o,1,3)];
        end

        function pose = task2pose(obj,task)
            % TASK2POSE converts a task configuration (6-element array) to
            % its corresponding pose (element of SE(3))
            %   pose = obj.task2pose(task)
            %
            %   Input(s)
            %       task - 6x1 array defining a task configuration
            %
            %       task = [d] (3-element translation parameterization)
            %              [k] (3-element rotation parameterization)
            %
            %   Output(s)
            %       pose - 4x4 array defining the end-effector pose
            %              relative to the base frame (element of SE(3))

            % TODO - check task parameterization input

            % Isolate rotation and translation parameters
            d_e2o = task(1:3);
            k_e2o = task(4:6);

            % Map rotation parametization to rotation matrix
            R_e2o = rotationVectorToMatrix(-k_e2o);

            % Package pose
            H_e2o = eye(4);
            H_e2o(1:3,1:3) = R_e2o;
            H_e2o(1:3,4) = d_e2o;
            pose = H_e2o;
        end

        function addJointHistory(obj)
            % ADDJOINTHISTORY appends the current joint configuration to
            % the JointHistory property for use with the "Undo" method.

            % Add element to joint history
            jointsAll = obj.JointHistory;

            % Get current joint configuration
            q = obj.Joints;

            % Append joint configuration
            jointsAll(:,end+1) = q;

            % Update joint history
            obj.JointHistory = jointsAll;
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
                    cTimeout = obj.Client.Timeout;
                otherwise
                    cTimeout = timeout;
            end

            obj.Timeout = timeout;

            if timeout ~= cTimeout
                fprintf('TCP/IP client timeout value does not match *.Timeout.');
            end
        end

        % Connection Timeout
        function set.ConnectTimeout(obj,timeout)
             switch lower( class(obj.Client) )
                case 'tcpclient'
                    cTimeout = obj.Client.ConnectTimeout;
                otherwise
                    cTimeout = timeout;
            end

            obj.ConnectTimeout = timeout;

            if timeout ~= cTimeout
                fprintf('TCP/IP client connection timeout value does not match *.ConnectTimeout.');
            end
        end

    % WaitOn
    function waitOn = get.WaitOn(obj)
        waitOn = obj.WaitOn;
    end

    function set.WaitOn(obj,waitOn)
        if ~islogical(waitOn)
            switch waitOn
                case 0
                    waitOn = false;
                case 1
                    waitOn = true;
                otherwise
                    error('Unrecognized WaitOn "%s". Please use a binary value.',waitOn);
            end
        end
        obj.WaitOn = waitOn;
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

    % Joint Acceleration
    function jointAcc = get.JointAcc(obj)
        jointAcc = obj.JointAcc;
    end

    function set.JointAcc(obj,jointAcc)
        % TODO - Update to make model specific, this may be robot specific
        if jointAcc < 0
            warning('Joint acceleration must be between 0 and 80*pi rad/sec^2.  Setting to 0 rad/sec^2.');
            jointAcc = 0;
        end
        if jointAcc > 80*pi
            warning('Joint acceleration must be between 0 and 80*pi rad/sec^2. Setting to 80*pi rad/sec^2.');
            jointAcc = 80*pi;
        end
        obj.JointAcc = jointAcc;
    end

    % Joint velocity
    function jointVel = get.JointVel(obj)
        jointVel = obj.JointVel;
    end

    function set.JointVel(obj,jointVel)
        % TODO - Update to make model specific, this may be robot specific
        if jointVel < 0
            warning('Joint velocity must be between 0 and 2*pi rad/sec. Setting to 0 rad/sec.');
            jointVel = 0;
        end
        if jointVel > 2*pi
            warning('Joint velocity must be between 0 and 2*pi rad/sec. Setting to 2*pi rad/sec.');
            jointVel = 2*pi;
        end
        obj.JointVel = jointVel;
    end

    % Task acceleration
    function taskAcc = get.TaskAcc(obj)
        taskAcc = obj.TaskAcc;
    end

    function set.TaskAcc(obj,taskAcc)
        if taskAcc < 0
            warning('Task acceleration must be between 0 and 150,000 mm/sec^2. Setting to 0 mm/sec^2.');
            taskAcc = 0;
        end
        if taskAcc > 150000
            warning('Task acceleration must be between 0 and 150,000 mm/sec^2. Setting to 150,000 mm/sec^2.');
            taskAcc = 150000;
        end
        obj.TaskAcc = taskAcc;
    end

    % Task velocity
    function taskVel = get.TaskVel(obj)
        taskVel = obj.TaskVel;
    end

    function set.TaskVel(obj,taskVel)
        if taskVel < 0
            warning('Task velocity must be between 0 and 3,000 mm/sec. Setting to 0 mm/sec.');
            taskVel = 0;
        end
        if taskVel > 3000
            warning('Task velocity must be between 0 and 3,000 mm/sec. Setting to 3,000 mm/sec.');
            taskVel = 3000;
        end
        obj.TaskVel = taskVel;
    end


    function joints = get.Joints(obj)
        % Get current joint configuration of the hardware
        % Joints - 1x6 array containing joint values (radians)
        msg = 'getarmjoints';
        obj.sendMsg(msg);
        joints = obj.receiveMsg(6,'array');
        joints = reshape(joints,[],1);

        % Check joint configuration against limits
        q_lims = obj.JointPositionLimits;
        tf_min = joints < q_lims(:,1);
        tf_max = joints > q_lims(:,2);
        tf_nan = isnan(joints);

        if nnz(tf_min) > 0 || nnz(tf_max) > 0 || nnz(tf_nan)
            strJlims  = fcnJointLimitsSTR(joints,q_lims,tf_min,tf_max,tf_nan);
            strReinit = fcnFlushBufferSTR(obj);
            str = sprintf(...
                ['The joint configuration returned from the ',...
                'controller exceeds the robot joint limits:\n%s%s'],...
                strJlims,strReinit);

            warning(str);

            joints = nan(6,1);
        end
    end

    function set.Joints(obj,joints)
        % Set the joint configuration of the simulation
        % movej([0,1.57,-1.57,3.14,-1.57,1.57],a=1.4, v=1.05, t=0, r=0)
        joints = reshape(joints,[],1);

        % Check joint configuration against limits
        q_lims = obj.JointPositionLimits;
        tf_min = joints < q_lims(:,1);
        tf_max = joints > q_lims(:,2);
        tf_nan = isnan(joints);

        if nnz(tf_min) > 0 || nnz(tf_max) > 0 || nnz(tf_nan)
            strJlims  = fcnJointLimitsSTR(joints,q_lims,tf_min,tf_max,tf_nan);
            str = sprintf(...
                ['The joint configuration specified ',...
                'exceeds the robot joint limits:\n%s\nIgnoring Command.\n'],...
                strJlims);

            warning(str);
            return
        end

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
        rsp = obj.receiveMsg(1,'value');

        % Wait for move
        if obj.WaitOn
            obj.WaitForMove;
        end
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
        task = obj.receiveMsg(6,'array');
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
        rsp = obj.receiveMsg(1,'value');

        % Wait for move
        if obj.WaitOn
            obj.WaitForMove;
        end
    end

    % Grip Position
    function gPos = get.GripPosition(obj)
        msg = 'get pos';
        obj.sendMsg(msg);
        gPos_uint8 = obj.receiveMsg(1,'value');
        gPos = gPos_uint8 * (52/255);

        % Update grip position
        obj.GripHistory = gPos;
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
        msg = sprintf('set pos %d',gPos_uint8);
        obj.sendMsg(msg);

        % Wait for move
        if obj.WaitOn
            obj.WaitForGrip;
        end
    end

    % Grip Speed
    function gSpeed = get.GripSpeed(obj)
        % TODO - figure out units and conversion for grip speed
        msg = 'get spe';
        obj.sendMsg(msg);
        gSpeed = obj.receiveMsg(1,'value');

        gSpeed = (gSpeed./255)*100; % Convert to percent
    end

    function set.GripSpeed(obj,gSpeed)
        % Set gripper speed as a percentage
        if gSpeed > 100
            warning('Gripper speed cannot exceed 100%');
            gSpeed = 100;
        end
        if gSpeed < 0
            warning('Gripper speed cannot be below 0%');
            gSpeed = 0;
        end

        gSpeed = gSpeed/100; % convert to decimal

        gSpeed = round( gSpeed*255 );

        msg = sprintf('set spe %d',uint8(gSpeed));
        obj.sendMsg(msg);
    end

    % Grip Force
    function gForce = get.GripForce(obj)
        % TODO - figure out units and conversion for grip force
        msg = sprintf('get for');
        obj.sendMsg(msg);
        gForce = obj.receiveMsg(1,'value');

        gForce = (gForce./255)*100; % Convert to percent
    end

    function set.GripForce(obj,gForce)
        % Set gripper speed as a percentage
        if gForce > 100
            warning('Gripper force cannot exceed 100%');
            gForce = 1;
        end
        if gForce < 0
            warning('Gripper force cannot be below 0%');
            gForce = 0;
        end

        gForce = gForce/100; % convert to decimal

        gForce = round( gForce*255 );

        msg = sprintf('set for %d',uint8(gForce));
        obj.sendMsg(msg);
    end

    function toolpose = get.ToolPose(obj)
        % Get the current tool pose of the hardware
        % ToolPose - 4x4 homogeneous transform representing the tool
        %            pose relative to the world frame
        H_e2o = obj.Pose;
        H_t2e = obj.FrameT;
        toolpose = H_e2o * H_t2e;
    end

    function set.ToolPose(obj,toolpose)
        % Set the current tool pose of the simulation
        H_t2o = toolpose;
        H_t2e = obj.FrameT;
        H_e2o = H_t2o * invSE(H_t2e);
        obj.Pose = H_e2o;
    end

    % ToolTask - 6x1 task configuration for the tool frame relative to
    %            the base frame [tran; rot]
    function tooltask = get.ToolTask(obj)
        % Get tool pose
        H_t2o = obj.ToolPose;
        % Convert to task space
        tooltask = obj.pose2task(H_t2o);
    end

    function set.ToolTask(obj,tooltask)
        % Convert to tool pose
        H_t2o = obj.task2pose(tooltask);
        % Update tool pose
        obj.ToolPose = H_t2o;
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

        % Update body-fixed tool frame Jacobian
        symFkin_t = obj.symFkin_e*frameT;
        obj.fcnJacobian_t = ...
            calculateJacobian(obj.symJoints,symFkin_t,false,...
            'Reference','Body','Order','TranslationRotation');
    end

    function set.JointHistory(obj,allJoints)
        % Set the history for undo method
        n = 50; % Limit size of history
        % alljoints(:,end+1) = joints;
        if size(allJoints,2) > 50
            allJoints(:,1) = [];
        end
        obj.JointHistory = allJoints;
    end

    function set.GripHistory(obj,gripPos)
        % Update grip history
        obj.GripHistory = gripPos;
    end

    function gripPos = get.GripHistory(obj)
        if isempty(obj.GripHistory)
            gripPos = inf;
        else
            gripPos = obj.GripHistory;
        end
    end
    % GetAccess ------------------------------------------------------

    % URmodel - Specified type of Universal Robot Manipulator
    function urmodel = get.URmodel(obj)
        urmodel = obj.URmodel;
    end

    % Jacobian_o - Base frame referenced Jacobian for current joint configuration [JTran; JRot]
    function jacobian_o = get.Jacobian_o(obj)
        q = obj.Joints;
        jacobian_o = obj.fcnJacobian_o(q);
    end

    % Jacobian_e - End-effector referenced Jacobian for current joint configuration [JTran; JRot]
    function jacobian_e = get.Jacobian_e(obj)
        q = obj.Joints;
        jacobian_e = obj.fcnJacobian_e(q);
    end

    % Jacobian_t - Tool frame referenced Jacobian for current joint configuration [JTran; JRot]
    function jacobian_t = get.Jacobian_t(obj)
        q = obj.Joints;
        jacobian_t = obj.fcnJacobian_t(q);
    end

    % Fkin_e - Forward kinematics for current joint configuration (end-effector relative to base)
    function fkin_e = get.Fkin_e(obj)
        q = obj.Joints;
        fkin_e = obj.fcnFkin_e(q);
    end

    % Fkin_t - Forward kinematics for current joint configuration (end-effector relative to base)
    function fkin_t = get.Fkin_t(obj)
        q = obj.Joints;
        fkin_t = obj.fcnFkin_t(q);
    end

    % DHtable - DH table associated with robot
    % TODO - allow user to calibrate robot to correct DH table
    function dhtable = get.DHtable(obj)
        q = obj.Joints;
        dhtable = obj.fcnDHtable(q);
    end

end % end methods
end % end classdef

%% Internal functions
function str = fcnFlushBufferSTR(obj)
str = sprintf('\nConsider flushing the URQt buffer:\n');
str = sprintf('%s\tur.FlushBuffer\n',str);
end

function str = fcnJointLimitsSTR(joints,q_lims,tf_min,tf_max,tf_nan)
str = [];

nJnts = numel(joints);
if nJnts ~= 6
    str = sprintf('%s\tTotal values returned: %d, Total values expected: 6\n',nJnts);
    return
end

for i = 1:nJnts
    if tf_min(i)
        str = sprintf('%s\tJoint %d:\n',str,i);
        str = sprintf('%s\t\tCurrent Value: %f\n',str,joints(i));
        str = sprintf('%s\t\t  Lower Limit: %f\n',str,q_lims(i,1));
    end
    if tf_max(i)
        str = sprintf('%s\tJoint %d:\n',str,i);
        str = sprintf('%s\t\tCurrent Value: %f\n',str,joints(i));
        str = sprintf('%s\t\t  Upper Limit: %f\n',str,q_lims(i,2));
    end
    if tf_nan(i)
        str = sprintf('%s\tJoint %d:\n',str,i);
        str = sprintf('%s\t\tCurrent Value: %f (Not a Number)\n',str,joints(i));
    end
end
end

function tcpClients = findTcpClients()
% Find all TCP clients
tcpClients = [];
allVars = evalin('base', 'whos'); % Get all variables in the base workspace

for i = 1:length(allVars)
    varName = allVars(i).name;
    try
        varValue = evalin('base', varName);
        if isa(varValue, 'tcpclient') && isvalid(varValue) && strcmp(varValue.Status, 'open')
            tcpClients = [tcpClients, varValue];
        end
    catch ME
        % Ignore errors that might occur when evaluating variables
        % that are not accessible or have issues.
        disp(['Warning: Error evaluating variable: ', varName, '. Error message: ', ME.message])

    end
end

end