function varargout = URQtToolboxVer
% URQTTOOLBOXVER displays the Universal Robot Toolbox for MATLAB 
% leveraging Qt information.
%   URQTTOOLBOXVER displays the information to the command prompt.
%
%   A = URQTTOOLBOXVER returns in A the sorted struct array of  
%   version information for the Piecewise Polynomial Toolbox.
%     The definition of struct A is:
%             A.Name      : toolbox name
%             A.Version   : toolbox version number
%             A.Release   : toolbox release string
%             A.Date      : toolbox release date
%
%   M. Kutzer 26Mar2021, USNA

% Updates:
%   26Mar2021 - Removed "master" reference from update, check for master
%               branch for support toolboxes in intall. 
%   01Apr2021 - Updated class to include model, Jacobian, and DH table
%   12Apr2021 - Documentation updates
%   20May2021 - Added isGripMoving and WaitForGrip methods
%   06Oct2021 - Corrected Joint/Task Vel/Acc limits and added 2-second
%               pause to *.Initialize to account for QtEXE startup
%   27Jan2022 - Added joint limit checks
%   27Jan2022 - Removed CB controller models from viable UR options

A.Name = 'Universal Robot Qt Toolbox';
A.Version = '1.0.6';
A.Release = '(R2021a)';
A.Date = '27-Jan-2022';
A.URLVer = 1;

msg{1} = sprintf('MATLAB %s Version: %s %s',A.Name, A.Version, A.Release);
msg{2} = sprintf('Release Date: %s',A.Date);

n = 0;
for i = 1:numel(msg)
    n = max( [n,numel(msg{i})] );
end

fprintf('%s\n',repmat('-',1,n));
for i = 1:numel(msg)
    fprintf('%s\n',msg{i});
end
fprintf('%s\n',repmat('-',1,n));

if nargout == 1
    varargout{1} = A;
end