% Shutdown Matlb->ROS if started, then start again. If there is a problem
% make sure roscore has been run
clear all;
try 
    rosshutdown
end
rosinit('138.25.49.44')

% Check topics
rostopic list
% Make sure at least the following are available (otherwise make sure dobot
% and openni cameras are started
% /dobot_magician/state
% /camera/depth/image
% /camera/rgb/image_color
% /rosout
% /rosout_agg 


%% Get and Show Color Images
rgbRawSub = rossubscriber('/camera/rgb/image_color');
pause(1);
image_h = imshow(readImage(rgbRawSub.LatestMessage));
% Make the figure large
set(gcf,'units','normalized','outerposition',[0 0 1 1]);
title('RGB image');
% Now loop through and update the image data with the latest image data
tic
for i = 1:100
% while 1
    image_h.CData = readImage(rgbRawSub.LatestMessage);
    drawnow;
    toc;
end

%% Get and Show Dept Images
depthRawSub = rossubscriber('/camera/depth/image');
pause(1);
msg = depthRawSub.LatestMessage;
img = readImage(msg);
image_h = imshow(img);
% Make the figure large
set(gcf,'units','normalized','outerposition',[0 0 1 1])
title('Depth image');
% Now loop through and update the image data with the latest image data
tic
for i = 1:100
    image_h.CData = readImage(depthRawSub.LatestMessage);
    drawnow;
    toc;
end

%% Dobot: Check messages
% Check the messages can be created: Create a state message
msg = rosmessage('dobot_magician/State');
msg
msg.Header
msg.EndpointPos

% Check the messages can be created: Create a position message
cartMsg = rosmessage('dobot_magician/SetPosCart')
cartMsg.Pos

%% Dobot: Get State
% Assuming that the custom messages are setup, roscore and the dobot node
% is started remotely, and /dobot/state is shown when rostopic list is
% called in Matlab. Then you can check the Dobot arm state with  
dobotStateSub = rossubscriber('/dobot_magician/state');
pause(1);
receive(dobotStateSub,10)
msg = dobotStateSub.LatestMessage
msg.Header
msg.Pos
msg.JointAngles

%% Dobot: Control joints 1-3

UP = [0,0.1,0.5,0];
DOWN = [0,1.1,1.1,0];
Part_1 = [-0.85,1.1,1.1,0]; 
Part_2 = [-0.65,1.49,0.60,0]; 
Part_3 = [0.65,1.3,0.9,0]; 
Part_4 = [0.85,1.1,1.1,0]; 
Above_Part_1 = [-0.85,1,1,0]; 
Above_Part_2 = [-0.65,1.39,0.50,0]; 
Above_Part_3 = [0.65,1.2,0.8,0]; 
Above_Part_4 = [0.85,1,1,0];

PATH = {UP,Above_Part_1,Part_1,Above_Part_1,UP,DOWN,UP,DOWN,UP,Above_Part_1,Part_1,Above_Part_1 ...
    ,UP,Above_Part_2,Part_2,Above_Part_2,UP,DOWN,UP,DOWN,UP,Above_Part_2,Part_2,Above_Part_2 ...
    ,UP,Above_Part_3,Part_3,Above_Part_3,UP,DOWN,UP,DOWN,UP,Above_Part_3,Part_3,Above_Part_3 ...
    ,UP,Above_Part_4,Part_4,Above_Part_4,UP,DOWN,UP,DOWN,UP,Above_Part_4,Part_4,Above_Part_4 ...
    }
PUMP = {0,0,1,1,1,0,0,1,1,1,0,0 ...
    ,0,0,1,1,1,0,0,1,1,1,0,0 ...
    ,0,0,1,1,1,0,0,1,1,1,0,0 ...
    ,0,0,1,1,1,0,0,1,1,1,0,0 ...
    }

% To move the joints of the arm and ensure completion
msgSetPos = rosmessage('dobot_magician/SetPosAngRequest');
msgSetPos.JointAngles = UP;
srv = rossvcclient('/dobot_magician/joint_angs');
srv.call(msgSetPos);

pause(2)
dobotStateSub.LatestMessage;
% dobotStateSub.LatestMessage.JointAngles
s = size(PATH)
for i=1:(s(2))
    % Note that once the above is called you could simply call the following to change the rear arm angle
    msgSetPos.JointAngles = PATH{i};
    srv.call(msgSetPos);

    pause(2)
    dobotStateSub.LatestMessage;
    % dobotStateSub.LatestMessage.Pos
    
    if PUMP{i} == 1
        msg2 = rosmessage('dobot_magician/SetPumpRequest');        
        msg2.Pump = 1; 
        eeSrv = rossvcclient('/dobot_magician/pump'); 
        eeSrv.call(msg2)
    elseif PUMP{i} == 0
        msg2.Pump =0; 
        eeSrv.call(msg2)
    end
    
end

%% msgSetPos.JointAngles = [0,0.1,0.5,0]; UP
%% msgSetPos.JointAngles = [0,1.1,1.1,0]; DOWN
%% msgSetPos.JointAngles = [-0.85,1.1,1.1,0]; Part_1
%% msgSetPos.JointAngles = [-0.65,1.49,0.60,0]; Part_2
%% msgSetPos.JointAngles = [0.68,1.3,0.9,0]; Part_3
%% msgSetPos.JointAngles = [0.85,1.1,1.1,0]; Part_4

%%

% To move the joints of the arm and ensure completion
msgSetPos = rosmessage('dobot_magician/SetPosAngRequest');
msgSetPos.JointAngles = UP
srv = rossvcclient('/dobot_magician/joint_angs');
srv.call(msgSetPos);

pause(2)
dobotStateSub.LatestMessage;
% dobotStateSub.LatestMessage.JointAngles

% Note that once the above is called you could simply call the following to change the rear arm angle
% msgSetPos.JointAngles = Above_Part_3;
srv.call(msgSetPos);

pause(2)
dobotStateSub.LatestMessage;
% dobotStateSub.LatestMessage.Pos

%% To turn the pump on 
msg2 = rosmessage('dobot_magician/SetPumpRequest');        
msg2.Pump = 1; 
eeSrv = rossvcclient('/dobot_magician/pump'); 
eeSrv.call(msg2) 

%% To turn the pump off
msg2.Pump =0; 

eeSrv.call(msg2)

%% Dobot catesian control
% To get the current pose in catesian space
dobotPosCartSub = rossvcclient('/dobot_magician/cart_pos','Timeout', 2);
dobotPosCartMsg = rosmessage(dobotPosCartSub);

dobotStateSub = rossubscriber('/dobot_magician/state');
receive(dobotStateSub,2)
msg = dobotStateSub.LatestMessage;
dobotPosCartMsg.Pos = msg.Pos;

dobotPosCartMsg.Pos.X = dobotPosCartMsg.Pos.X - 0.05;
dobotPosCartSub.call(dobotPosCartMsg)





%% To turn the pump on 
function [] = pumpOn()
    msg2 = rosmessage('dobot_magician/SetPumpRequest');        
    msg2.Pump = 1; 
    eeSrv = rossvcclient('/dobot_magician/pump'); 
    eeSrv.call(msg2)
end

%% To turn the pump off
function [] = pumpOff()
    msg2.Pump =0; 
    eeSrv.call(msg2)
end