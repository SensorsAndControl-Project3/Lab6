%% Connect Remotely to ROS from Matlab %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rosinit('192.168.0.103');
% find(cell2mat(strfind(rosmsg('list'),'dobot')),1)
rostopic list
rosservice list

%% Get and Show Colour Images %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rgbSub = rossubscriber('/camera/rgb/image_color');
pause(1);
image_h = imshow(readImage(rgbSub.LatestMessage));

%% Make the figure large

set(gcf,'units','normalized','outerposition',[0 0 1 1]);

%% Now loop through and update the image data with the latest image data (ctrl + c to stop)

tic
while 1
 image_h.CData = readImage(rgbSub.LatestMessage);
 drawnow;
 toc;
end

%% Get and Show Depth Images %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

depthSub = rossubscriber('/camera/depth/image');
pause(1);
msg = depthSub.LatestMessage;
img = readImage(msg);
depthImage_h = imshow(img)

%% Make the figure large

set(gcf,'units','normalized','outerposition',[0 0 1 1])

%% Now loop through and update the image data with the latest image data (ctrl + c to stop)

tic
while 1
 depthImage_h.CData = readImage(depthSub.LatestMessage);
 drawnow;
 toc;
end

%% Get and Show Point cloud

% See the example on UTSOnline ColorPointTest.m

%% Get Dobot State and Control from Matlab

stateSub = rossubscriber('/dobot_magician/state');
receive(stateSub,2)
msg = stateSub.LatestMessage;

%% To move the joints of the arm and ensure completion (remember angles are in radians)

msg = rosmessage('dobot_magician/SetPosAngRequest');
msg.RearArmAngle = 0.8304;
msg.ForeArmAngle = 1.3138;
msg.BaseAngle = 0;
srv = rossvcclient('/dobot_magician/joint_angs');
srv.call(msg)

%% Note that once the above is called you could simply call the following to change the rear arm angle

msg.BaseAngle = -pi/4;
srv.call(msg)

%% To request the robot to move in Cartesian space (all in meters)

cartSvc = rossvcclient('dobot/cartesian');
cartMsg = rosmessage(cartSvc);
cartMsg.Pos.X = 0.2;
cartMsg.Pos.Y = 0.1;
cartMsg.Pos.Z = -0.02;
cartSvc.call(cartMsg);

%% To turn the pump on. Note: this may cause the robot to freeze, if so, logout of the remote lab and then back again to fix this.

msg2 = rosmessage('dobot_ros/EECtrlRequest');
msg2.Pump = 1;
eeSrv = rossvcclient('/dobot/ee_ctrl');
eeSrv.call(msg2)

%% Then to turn it off again

msg2.Pump = 0;
eeSrv.call(msg2);

%% End Session %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

rosshutdown

%%

dobot_state_sub = rossubscriber('/dobot_magician/state');
receive(dobot_state_sub,2)
state_msg = dobot_state_sub.LatestMessage;

state_msg.JointAngles
%%
joint_srv = rossvcclient('/dobot_magician/joint_angs');
%joint_msg = rosmessage('dobot_magician/SetPosAngRequest');
joint_msg = rosmessage(joint_srv)

state_msg.JointAngles
%% 
pump_srv = rossvcclient('/dobot_magician/pump');
%joint_msg = rosmessage('dobot_magician/SetPosAngRequest');
pump_msg = rosmessage(pump_srv);

pump_msg.Pump = 0;

pump_srv.call(pump_msg);

%%
joint_msg.JointAngles(1) = -pi/6;
joint_msg.JointAngles(2) = pi/3;
joint_msg.JointAngles(3) = pi/4;
joint_msg.JointAngles(4) = pi/2;

joint_srv.call(joint_msg);


%% home
joint_msg.JointAngles(1) = 0.0;
% joint_msg.JointAngles(2) = ((pi*sqrt(2))/6);
% joint_msg.JointAngles(3) = ((pi*sqrt(2))/6);
joint_msg.JointAngles(2) = (deg2rad(45));
joint_msg.JointAngles(3) = (deg2rad(45));
joint_msg.JointAngles(4) = (deg2rad(0));
% joint_msg.JointAngles(4) = (deg2rad(150));
% joint_msg.JointAngles(4) = (deg2rad(-150));

joint_srv.call(joint_msg);

%% Max
joint_msg.JointAngles(1) = (deg2rad(125));
% joint_msg.JointAngles(1) = -9*pi/12;
joint_msg.JointAngles(2) = 1.66185;
joint_msg.JointAngles(3) = 1.55456;
joint_msg.JointAngles(4) = (deg2rad(150));

joint_srv.call(joint_msg);

%% Min
joint_msg.JointAngles(1) = (deg2rad(-125));
joint_msg.JointAngles(2) = -0.16691;
joint_msg.JointAngles(3) = -0.32792;
joint_msg.JointAngles(4) = (deg2rad(-150));

joint_srv.call(joint_msg);
%%
joint_msg.JointAngles(1) = pi/3; 
joint_srv.call(joint_msg)

%%
cart_srv = rossvcclient('dobot_magician/cart_pos');
cart_msg = rosmessage(cart_srv); 

cart_msg.Pos.X = 0.2;
cart_msg.Pos.Y = 0.0;
cart_msg.Pos.Z = 0.07;

cart_srv.call(cart_msg)

%%
joint_msg.JointAngles(1) = 0;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;

joint_srv.call(joint_msg);
