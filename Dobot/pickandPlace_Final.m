% DoBot controller for controlling the real DoBot
clf % Clear Figure
Initialise = 0;
% if 1 run initialisation
% if 0 don't run initialisation
if Initialise == 1
    % Initialise ros services and ros topics
    close all
    clear all
    set(0,'DefaultFigureWindowStyle','docked')
    clc
    
    % Try rosshutdown before rosinit
    try rosshutdown; end
    % ROS init running on this Computer
    rosinit
    % check what ros topic are being received
    rostopic list
    % check what ros services are being received
    rosservice list
    
    % Gives joint angles
    dobot_state_sub = rossubscriber('/dobot_magician/state');
    receive(dobot_state_sub,2);
    state_msg = dobot_state_sub.LatestMessage;
    state_msg.JointAngles;
    
    % sets up the ros client
    joint_srv = rossvcclient('/dobot_magician/joint_angs');
    joint_msg = rosmessage(joint_srv)
    joint_msg.JointAngles
    
    % dobot_magiciam pump control
    pump_srv = rossvcclient('/dobot_magician/pump');
    pump_msg = rosmessage(pump_srv);
    pump_msg.Pump = 0;
    try pump_srv.call(pump_msg); end
    
    % Initialise Pose starting from the back
    joint_msg.JointAngles(1) = 0;
    joint_msg.JointAngles(2) = 0;
    joint_msg.JointAngles(3) = 0;
    joint_msg.JointAngles(4) = 0;
    joint_srv.call(joint_msg);
end
% init pump off
pump(0);

% Initialise Pose starting from the back
joint_msg.JointAngles(1) = 0;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);
pause(0.5);
            
% move over the right side position 1 to take scan
joint_msg.JointAngles(1) = pi/2;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% read from the camera for the first image

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% move over the right side to position 2 to take scan
joint_msg.JointAngles(1) = pi/2;
joint_msg.JointAngles(2) = 5;
joint_msg.JointAngles(3) = -5;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% read from the camera for the second image

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate the X and Y values of each shape

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% these will become the inputs from the code above
% Scattered Shapes position [X,Y]
ScatteredPose1 = [-72,(318-30)];
ScatteredPose2 = [55,(318-30)];
ScatteredPose3 = [72,(267-30)];
ScatteredPose4 = [-60,(255-30)];

% Scattered Shapes Angles
ScatteredAngle1 = rad2deg(atan(ScatteredPose1(1)/ScatteredPose1(2)));
ScatteredAngle2 = rad2deg(atan(ScatteredPose2(1)/ScatteredPose2(2)));
ScatteredAngle3 = rad2deg(atan(ScatteredPose3(1)/ScatteredPose3(2)));
ScatteredAngle4 = rad2deg(atan(ScatteredPose4(1)/ScatteredPose4(2)));

display(['ScatteredAngle1: ', num2str(ScatteredAngle1)]);
display(['ScatteredAngle2: ', num2str(ScatteredAngle2)]);
display(['ScatteredAngle3: ', num2str(ScatteredAngle3)]);
display(['ScatteredAngle4: ', num2str(ScatteredAngle4)]);

disp(' ');

% Sorted Shapes position [X,Y]
SortedPose1 = [6,(326-30)];
SortedPose2 = [2,(242-30)];
SortedPose3 = [-54,(339-30)];
SortedPose4 = [69,(327-30)];

% Sorted Shapes Angles
SortedAngle1 = rad2deg(atan(SortedPose1(1)/SortedPose1(2)));
SortedAngle2 = rad2deg(atan(SortedPose2(1)/SortedPose2(2)));
SortedAngle3 = rad2deg(atan(SortedPose3(1)/SortedPose3(2)));
SortedAngle4 = rad2deg(atan(SortedPose4(1)/SortedPose4(2)));

display(['SortedAngle1: ', num2str(SortedAngle1)]);
display(['SortedAngle2: ', num2str(SortedAngle2)]);
display(['SortedAngle3: ', num2str(SortedAngle3)]);
display(['SortedAngle4: ', num2str(SortedAngle4)]);

disp(' ');

RotationAngle1 = ScatteredAngle1 - SortedAngle1 - 45;
RotationAngle2 = ScatteredAngle2 - SortedAngle2 - 90;
RotationAngle3 = ScatteredAngle3 - SortedAngle3;
RotationAngle4 = ScatteredAngle4 - SortedAngle4;

display(['RotationAngle1: ', num2str(RotationAngle1)]);
display(['RotationAngle2: ', num2str(RotationAngle2)]);
display(['RotationAngle3: ', num2str(RotationAngle3)]);
display(['RotationAngle4: ', num2str(RotationAngle4)]);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Include the depth from the camera to the table
DepthZ = 200;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% pumpState = 0 off
% pumpState = 1 on
pumpState = 0;

% PalletSide = 0 right
% PalletSide = 1 left
PalletSide = 1;

for pickPlaceNum = 1 : 4
    % 1 = Shape 1
    % 2 = Shape 2
    % 3 = Shape 3
    % 4 = Shape 4
    
    for sideNum = 1 : 2
        
        if sideNum == 1
            
            % move over the right side
            joint_msg.JointAngles(1) = pi/2;
            joint_msg.JointAngles(2) = 0;
            joint_msg.JointAngles(3) = 0;
            joint_msg.JointAngles(4) = 0;
            joint_srv.call(joint_msg);
            ImageJointAngles = [pi/2,0,0,0,0];
            pumpState = 1;
            PalletSide = 0;
            
        elseif sideNum == 2
            
            % move over the left side
            joint_msg.JointAngles(1) = -pi/2;
            joint_msg.JointAngles(2) = 0;
            joint_msg.JointAngles(3) = 0;
            joint_msg.JointAngles(4) = 0;
            joint_srv.call(joint_msg);
            ImageJointAngles = [-pi/2,0,0,0,0];
            pumpState = 0;
            PalletSide = 1;
            
        end
        joint_srv.call(joint_msg);
        
        pause;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Offset of the camera with respect to the end effector
        Camera2EndeffectorCoords = [X,Y,Z];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rotation = 0;
        
        if pickPlaceNum == 1 && sideNum == 1
            
            BoxCoords = [ScatteredPose1,DepthZ];
            
        elseif pickPlaceNum == 1  && sideNum == 2
            
            BoxCoords = [SortedPose1,DepthZ];
            rotation = RotationAngle1;
            
        elseif pickPlaceNum == 2  && sideNum == 1
            
            BoxCoords = [ScatteredPose2,DepthZ];
            
        elseif pickPlaceNum == 2  && sideNum == 2
            
            BoxCoords = [SortedPose2,DepthZ];
            rotation = RotationAngle2;
            
        elseif pickPlaceNum == 3 && sideNum == 1
            
            BoxCoords = [ScatteredPose3,DepthZ];
            
        elseif pickPlaceNum == 3  && sideNum == 2
            
            BoxCoords = [SortedPose3,DepthZ];
            rotation = RotationAngle3;
            
        elseif pickPlaceNum == 4  && sideNum == 1
            
            BoxCoords = [ScatteredPose4,DepthZ];
               
        elseif pickPlaceNum == 4  && sideNum == 2
            
            BoxCoords = [SortedPose4,DepthZ];
            rotation = RotationAngle4;
            
        end              
        
        BoxCoords = BoxCoords/1000; % box pick or box position to place
        BoxCoordinates = BoxCoords + Camera2EndeffectorCoords;
        AboveBoxCoordinates = [BoxCoordinates(1), BoxCoordinates(2), 0/1000];
        
        Dobot(BoxCoordinates, AboveBoxCoordinates, ImageJointAngles, PalletSide, pumpState, rotation);
        
        % move over the back side
        joint_msg.JointAngles(1) = 0;
        joint_msg.JointAngles(2) = 0;
        joint_msg.JointAngles(3) = 0;
        joint_msg.JointAngles(4) = 0;
        joint_srv.call(joint_msg);
    
    end
end

function Dobot(BoxCoordinates, AboveBoxCoordinates, ImageJointAnglesfn, PalletSidefn, pumpStatefn, rotation)

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
L13 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-45 85]),'offset',-pi/2);
L14 = Link('d',0,'a',-wristDist,'alpha',pi/2,'qlim',deg2rad([-90 90]),'offset',0);
L15 = Link('d',endEffectorDist,'a',0,'alpha',0,'qlim',deg2rad([-85 85]),'offset',0);

% model for ikine and ikcon
A = wristDist; % Wrist: 61.5 mm
B = endEffectorDist; % End Effector: 79.5 mm
C = sqrt((A^2)+(B^2));
theta = atan(B/A);
L21 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L22 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([5 80]),'offset',pi/2);
L23 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-45 85]),'offset',-pi/2);
L24 = Link('d',0,'a',-C,'alpha',0,'qlim',deg2rad([-90 90]),'offset',theta);

% Creating the Robot SerialLink Models
DoBotVisuModel = SerialLink([L11 L12 L13 L14 L15],'name','Visual DoBot');
DoBotCalcModel = SerialLink([L21 L22 L23 L24],'name','Calculations DoBot');
baseLocation = transl(0,0,0) * trotx(0) * troty(0) * trotz(0);
DoBotVisuModel.base = baseLocation;
DoBotCalcModel.base = baseLocation;
workspace = [-0.5 0.5 -0.5 0.5 -0.2 0.5];
scale = 0.5;

% Plotting the Visual Model
figure
DoBotVisuModel.plot(zeros(1,DoBotVisuModel.n),'workspace',workspace,'scale',scale, 'noarrow');
hold on
DoBotVisuModel.teach
% Specifying the coordinates that the robot arm need to move to
ImageEndEffectorPose = DoBotVisuModel.fkine(ImageJointAnglesfn)

Base2EndEffector = [ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)'];
if PalletSidefn == 0
    
    Camera2Object = [0,0,0;
        -AboveBoxCoordinates(2),-AboveBoxCoordinates(1),-AboveBoxCoordinates(3);
        -BoxCoordinates(2), -BoxCoordinates(1), -BoxCoordinates(3);
        -AboveBoxCoordinates(2),-AboveBoxCoordinates(1),-AboveBoxCoordinates(3);
        0,0,0];
    
elseif PalletSidefn == 1
    
    Camera2Object = [0,0,0;
        AboveBoxCoordinates(2),AboveBoxCoordinates(1),-AboveBoxCoordinates(3);
        BoxCoordinates(2), BoxCoordinates(1), -BoxCoordinates(3);
        AboveBoxCoordinates(2),AboveBoxCoordinates(1),-AboveBoxCoordinates(3);
        0,0,0];
    
end

Coordinates = (Base2EndEffector + Camera2Object)


if pumpStatefn == 1
    PumpOperation = [0,0,1,1,1];
elseif pumpStatefn == 0
    PumpOperation = [1,1,0,0,0];
end

% Creating a loop depending on the number of coordinates that the robot
% must move to as specified above
sizeof = size(Coordinates);
for x = 1:sizeof(1)
    
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
        setJointAngles(qMatrix(i,1),qMatrix(i,2),(qMatrix(i,3) + qMatrix(i,2)),qMatrix(i,4),count,rotation)
    end
    DoBotVisuModel.teach
    
    % set the pump
    pump(PumpOperation(x));
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

function [] = setJointAngles(j1,j2,j3,j4,count,rotation)

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
joint_msg.JointAngles(4) = rotation;
joint_srv.call(joint_msg);

end

% function [] = cartesianCoordiantesControl(j1,j2,j3,j4)
% % Cartesian Coordiantes Control
% cart_srv = rossvcclient('dobot_magician/cart_pos');
% cart_msg = rosmessage(cart_srv);
% cart_msg.Pos.X = 0.2;
% cart_msg.Pos.Y = 0.0;
% cart_msg.Pos.Z = 0.07;
% cart_srv.call(cart_msg)
% end

%% home

function Back()
%%
joint_msg.JointAngles(1) = 0;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
ImageJointAngles = [0,0,0,0,0];
joint_srv.call(joint_msg);
end

function Left()
%%
joint_msg.JointAngles(1) = pi/2;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
ImageJointAngles = [pi/2,0,0,0,0];
joint_srv.call(joint_msg);
end

function Right()
%%
joint_msg.JointAngles(1) = -pi/2;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
ImageJointAngles = [-pi/2,0,0,0,0];
joint_srv.call(joint_msg);
end

function [] = pumpOff()
%%
pump_srv = rossvcclient('/dobot_magician/pump');
pump_msg = rosmessage(pump_srv);
pump_msg.Pump = 0;
try pump_srv.call(pump_msg); end
end

function [] = pumpOn()
%%
pump_srv = rossvcclient('/dobot_magician/pump');
pump_msg = rosmessage(pump_srv);
pump_msg.Pump = 1;
try pump_srv.call(pump_msg); end
end
%%
