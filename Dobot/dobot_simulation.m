function [] = dobot_simulation()
% DoBot Simulation for controlling the real DoBot
close all
clear all
set(0,'DefaultFigureWindowStyle','docked')
clc

% Base = 137.5 mm
% Lower Arm:  135.0 mm
% Upper Arm: 147.5 mm
% Wrist: 61.5 mm
% End Effector: 79.5 mm

baseDist = 137.5/1000; % Base = 0.1375 m
lowerArmDist = 135.0/1000; % Lower Arm:  0.1350 m
upperArmDist = 147.5/1000; % Upper Arm: 0.1475 m
wristDist = 61.5/1000; % Wrist: 0.0615 m
endEffectorDist = 79.5/1000; % End Effector: 0.0795 m

% model for Visualisation and Rotation of endEffector
L11 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L12 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([-5 85]),'offset',pi/2);
L13 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-10 95]),'offset',-pi/2);
L14 = Link('d',0,'a',-wristDist,'alpha',pi/2,'qlim',deg2rad([-90 90]),'offset',0);
L15 = Link('d',endEffectorDist,'a',0,'alpha',0,'qlim',deg2rad([-85 85]),'offset',0);

% model for ikine and ikcon
A = wristDist; % Wrist: 61.5 mm
B = endEffectorDist; % End Effector: 79.5 mm
C = sqrt((A^2)+(B^2));
theta = atan(B/A);
L21 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L22 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([-5 85]),'offset',pi/2);
L23 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-10 95]),'offset',-pi/2);
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
DoBotVisuModel.plot(zeros(1,DoBotVisuModel.n),'workspace',workspace,'scale',scale, 'noarrow');
hold on

% Specifying the coordinates that the robot arm need to move to
Coordinates = [-0.08,0.12,-0.2;-0.2090,0.0000,0.1930;0.00,-0.3,0.05];

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
    for i = 2:1:s(1)
        poseDoBot = poseDoBotFunct(qMatrix(i,:));
        if rad2deg(max(abs(poseDoBot-pose))) > deg2rad(1)
            DoBotVisuModel.animate(poseDoBot);
            drawnow()
            pose = poseDoBotFunct(qMatrix(i,:));
        end
    end
    DoBotVisuModel.teach
end

end

function [output] = poseDoBotFunct(jointAngles)

j1 = jointAngles(1);
j2 = jointAngles(2);
j3 = jointAngles(3);
j4 = ((2*pi)-(j2 + pi)-(j3 + (pi/2))-(pi/2));
% j4 = jointAngles(4);
j5 = 0;

output = [j1,j2,j3,j4,j5];

end

function [qValues2] = visu2CalcqValues(coordinates, qValues, robotCalc)

qValues = qValues(1:4);
modelTrans = robotCalc.fkine(qValues);
% threeDimenPose = modelTrans(1:3,4);

actualTrans = [modelTrans(1,1:3),coordinates(1);
    modelTrans(2,1:3),coordinates(2);
    modelTrans(3,1:3),coordinates(3);
    modelTrans(4,1:3),1];
qValues2 = robotCalc.ikcon(actualTrans);
end

function [qMatrix] = modelCoordinates(coordinates, robotCalc, robotVisu)

qValues = robotVisu.getpos();
currentqValues = qValues(1:4);

actualTrans = transl(coordinates)* trotx(pi);
qValues = robotVisu.ikcon(actualTrans);
newqValues = visu2CalcqValues(coordinates, qValues, robotCalc);

qMatrix = jtraj(currentqValues,newqValues,100);

end


