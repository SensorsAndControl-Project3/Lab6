% DoBot controller for controlling the real DoBot

% if 1 run initialisation
% else don't run initialisation
Initialise = 0
clf
if Initialise == 1
    close all
    clear all
    set(0,'DefaultFigureWindowStyle','docked')
    clc
    
    try rosshutdown
    end
    
%     ipaddress = '192.168.31.141';
%     rosinit(ipaddress)
    rosinit
    rostopic list
    rosservice list
    
    % Gives joint angles
    pause(2)
    dobot_state_sub = rossubscriber('/dobot_magician/state');
%     receive(dobot_state_sub,2)
    state_msg = dobot_state_sub.LatestMessage;
    state_msg.JointAngles
    
    % sets up the ros client
    pause(2)
    joint_srv = rossvcclient('/dobot_magician/joint_angs');
    joint_msg = rosmessage(joint_srv)
    joint_msg.JointAngles
    
    % dobot_magiciam pump control
    pause(2)
    pump_srv = rossvcclient('/dobot_magician/pump');
    pump_msg = rosmessage(pump_srv);
    pump_msg.Pump = 0;
    try pump_srv.call(pump_msg); end
    
%     % Initialise Pose
%     joint_msg.JointAngles(1) = 0;
%     joint_msg.JointAngles(2) = 0;
%     joint_msg.JointAngles(3) = 0;
%     joint_msg.JointAngles(4) = 0;
%     joint_srv.call(joint_msg);
    
end

Dobot()

function Dobot()

% Base = 137.5 mm
% Lower Arm:  135.0 mm
% Upper Arm: 147.5 mm
% Wrist: 61.5 mm
% End Effector: 79.5 mm

count = 0;

baseDist = 137.5/1000; % Base = 0.1375 m
lowerArmDist = 135.0/1000; % Lower Arm:  0.1350 m
upperArmDist = 147.5/1000; % Upper Arm: 0.1475 m
wristDist = 61.5/1000; % Wrist: 0.0615 m
endEffectorDist = 79.5/1000; % End Effector: 0.0795 m

% model for Visualisation and Rotation of endEffector
L11 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L12 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([5 80]),'offset',pi/2);
% L13 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-5 85]),'offset',-pi/2);
L13 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([15 170]),'offset',-pi/2);
L14 = Link('d',0,'a',-wristDist,'alpha',pi/2,'qlim',deg2rad([-90 90]),'offset',0);
L15 = Link('d',endEffectorDist,'a',0,'alpha',0,'qlim',deg2rad([-85 85]),'offset',0);

% model for ikine and ikcon
A = wristDist; % Wrist: 61.5 mm
B = endEffectorDist; % End Effector: 79.5 mm
C = sqrt((A^2)+(B^2));
theta = atan(B/A);
L21 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L22 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([5 80]),'offset',pi/2);
L23 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([15 170]),'offset',-pi/2);
L23 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([15 170]),'offset',-pi/2);
L24 = Link('d',0,'a',-C,'alpha',0,'qlim',deg2rad([-90 90]),'offset',theta);

% Creating the Robot SerialLink Models
DoBotVisuModel = SerialLink([L11 L12 L13 L14 L15],'name','Visual DoBot');
DoBotCalcModel = SerialLink([L21 L22 L23 L24],'name','Calculations DoBot');
baseLocation = transl(0,0,0) * trotx(0) * troty(0) * trotz(0);
DoBotVisuModel.base = baseLocation;
DoBotCalcModel.base = baseLocation;
workspace = [-0.5 0.5 -0.5 0.5 -0.2 0.5];
scale = 0.5;

% q = readJointAnglesSetup()
% trans = DoBotCalcModel.fkine(q)
% xyz = trans(1:3,4)
% angles = poseDoBotFunct(q)
% DoBotVisuModel.teach(angles)

% Plotting the Visual Model
figure(1)
DoBotVisuModel.plot(zeros(1,DoBotVisuModel.n),'workspace',workspace,'scale',scale, 'noarrow');
hold on
DoBotVisuModel.teach
% Specifying the coordinates that the robot arm need to move to
% Coordinates = [-0.08,0.12,-0.2;-0.2090,0.0000,0.1930;0.00,-0.3,0.05];
% Coordinates = [-0.08,0.05,-0.1;-0.2090,0.0000,0.1930;0.00,-0.3,0.05];
% Coordinates = [-0.161,0.133,0.056;-0.000,-0.187,0.011;-0.212,-0.104,0.062;-0.180,0.139,0.139;-0.209,-0.000,0.193;-0.209,-0.035,-0.047];
% Coordinates = [-0.209,-0.035,-0.047];
% PumpOperation = [1,0,1,0,1,0];

% Coordinates = [-0.2090,0.0000,0.1930]
% Coordinates = [-0.212,0.0000,0.142]

% Coordinates = [-0.157,0.000,0.006]
% Coordinates = [-0.0160, 0.0000, 0.0241]
% Coordinates = [-0.1571,-0.0000,0.0059]   
    
% Coordinates = [-0.231,-0.0000,-0.1026;
%     -0.231,-0.0000,-0.0826;
%     -0.231,-0.0000,-0.0666;
%     -0.231,-0.0000,-0.0426;
%     -0.231,-0.0000,-0.0226;
%     -0.231,-0.0000,0.0026;
%     -0.231,-0.0000,0.0226;
%     -0.231,-0.0000,0.0426;
%     -0.231,-0.0000,0.0626;
%     -0.231,-0.0000,0.0826;
%     -0.231,-0.0000,0.1026;
%     -0.231,-0.0000,0.1226;
%     -0.231,-0.0000,0.1426;
%     -0.231,-0.0000,0.1626]

Coordinates = [-0.231,-0.0000,-0.1500];

% Creating a loop depending on the number of coordinates that the robot
% must move to as specified above
sizeof = size(Coordinates);
for x = 1:sizeof(1)
    
%     figure(2)
%     imageSub = rossubscriber('/camera/rgb/image_raw');
%     pause(1);
%     image_h = imshow(readImage(imageSub.LatestMessage));
    
    try pump(PumpOperation(x)); end 
    
    % the Coordinates that the robot is moveing to
    newCoordinates = Coordinates(x,:)
    
    % trajectory qJoint angles to get to the above Coordinates
    qMatrix = modelCoordinates(newCoordinates, DoBotCalcModel, DoBotVisuModel);
    
    % initialising variables
    s = size(qMatrix);
    poseDoBot = poseDoBotFunct(qMatrix(1,:));
    DoBotVisuModel.animate(poseDoBot);
    pose = poseDoBot;
    
    % Drawing the robot
    drawnow()
    
    DoBotVisuModel.delay = 0.1;
    %     for i = 2:1:s(1)
    for i = s(1):1:s(1)
        poseDoBot = poseDoBotFunct(qMatrix(i,:));
        DoBotVisuModel.animate(poseDoBot);
        drawnow();
        pose = poseDoBotFunct(qMatrix(i,:));
        setJointAngles(qMatrix(i,1),qMatrix(i,2),(qMatrix(i,3) + qMatrix(i,2)),qMatrix(i,4),count)
    end
    DoBotVisuModel.teach
end

end

% adjust so that It keeps the j4 vertical
function [output] = poseDoBotFunct(jointAngles)

j1 = jointAngles(1);
j2 = jointAngles(2);
j3 = jointAngles(3);
j4 = ((2*pi)-(j2 + pi)-(j3 + (pi/2))-(pi/2));
j5 = 0;

output = [j1,j2,j3,j4,j5];

end

function [qValues2] = visu2CalcqValues(qValues, robotCalc)

qValues = qValues(1:4);
modelTrans = robotCalc.fkine(qValues);
threeDimenPose = modelTrans(1:3,4);

actualTrans = [modelTrans(1,1:3),threeDimenPose(1);
    modelTrans(2,1:3),threeDimenPose(2);
    modelTrans(3,1:3),threeDimenPose(3);
    modelTrans(4,1:3),1];
qValues2 = robotCalc.ikcon(actualTrans);
end

function [qMatrix] = modelCoordinates(coordinates, robotCalc, robotVisu)

qValues = robotVisu.getpos();
currentqValues = qValues(1:4);

actualTrans = transl(coordinates)* trotx(pi);
qValues = robotVisu.ikcon(actualTrans);
newqValues = visu2CalcqValues(qValues, robotCalc);

qMatrix = jtraj(currentqValues,newqValues,100);

end

function [msg] = readJointAnglesSetup()
% read joint angles setup
dobot_state_sub = rossubscriber('/dobot_magician/state');
receive(dobot_state_sub,2)
state_msg = dobot_state_sub.LatestMessage;
msg = state_msg.JointAngles
end

function [msg] = readJointAngles()
% read Joint angles
joint_srv = rossvcclient('/dobot_magician/joint_angs');
joint_msg = rosmessage(joint_srv)
msg = state_msg.JointAngles
end

function [] = pump(onOff)
pump_srv = rossvcclient('/dobot_magician/pump');
pump_msg = rosmessage(pump_srv);
pump_msg.Pump = onOff;
try pump_srv.call(pump_msg); end
end

function [] = setJointAngles(j1,j2,j3,j4,count)

if count == 0
    count = count + 1;
    dobot_state_sub = rossubscriber('/dobot_magician/state');
    joint_srv = rossvcclient('/dobot_magician/joint_angs');
    joint_msg = rosmessage(joint_srv)
    
end

% Set Joint angles
joint_msg.JointAngles(1) = j1;
joint_msg.JointAngles(2) = j2;
joint_msg.JointAngles(3) = j3;
% joint_msg.JointAngles(4) = j4;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

end

function [] = cartesianCoordiantesControl(j1,j2,j3,j4)
% Cartesian Coordiantes Control
cart_srv = rossvcclient('dobot_magician/cart_pos');
cart_msg = rosmessage(cart_srv);
cart_msg.Pos.X = 0.2;
cart_msg.Pos.Y = 0.0;
cart_msg.Pos.Z = 0.07;
cart_srv.call(cart_msg)
end

