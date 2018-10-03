function [] = MovingDoBot()
%MOVINGDOBOT for controlling the real DoBot
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

L1 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L2 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([5 80]),'offset',pi/2);
L3 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([15 85]),'offset',-pi/2);
L4 = Link('d',0,'a',-wristDist,'alpha',pi/2,'qlim',deg2rad([-90 90]),'offset',0);
L5 = Link('d',endEffectorDist,'a',0,'alpha',0,'qlim',deg2rad([-85 85]),'offset',0);

%%

DoBot = SerialLink([L1 L2 L3 L4 L5],'name','DoBot');
baseLocation = transl(0,0,0) * trotx(0) * troty(0) * trotz(0);
DoBot.base = baseLocation;

workspace = [-0.5 0.5 -0.5 0.5 -0.2 0.5];
scale = 0.5;

DoBot.plot(zeros(1,DoBot.n),'workspace',workspace,'scale',scale, 'noarrow');
DoBot.teach()

qZeros = [0,0,0,0,0];

T = DoBot.fkine(qZeros)
poseIkine = DoBot.ikine(T,[0,0,0,0,0],[1,1,1,0,0,0])
poseIkcon = DoBot.ikcon(T)
hold on

qStruct = load ('dobot_q.mat');
qMatrix = qStruct.dobot_q;
s = size(qMatrix);
q = qMatrix;
givenPose = q(1,:);
poseDoBot = poseDoBotFunct(q(1,:));
DoBot.animate(poseDoBot);
waypoint = poseDoBot;
drawnow()
pose = poseDoBot;
count = 0;
DoBot.delay = 0.1;
mode = 0;
for i = 2:1:s(1)
    poseDoBot = poseDoBotFunct(q(i,:));
    if mode == 0
        poseDoBot = poseDoBotFunct(q(i,:));
        if waypoint(1) < poseDoBot
           mode = 1;
        else
            poseDoBot = poseDoBotFunct(q(i,:));
            waypoint = poseDoBot;
        end
    elseif mode == 1
        poseDoBot = poseDoBotFunct(q(i,:));
        if waypoint(1) > poseDoBot
           mode = 0;
        else
            poseDoBot = poseDoBotFunct(q(i,:));
            waypoint = poseDoBot;
        end
    end
    poseDoBot = poseDoBotFunct(q(i,:));
    poseDoBot1 = poseDoBotFunct(q(i-1,:));
    rad2deg(max(abs(poseDoBot-poseDoBot1)));
    poseDoBot = poseDoBotFunct(q(i,:));
    if rad2deg(max(abs(poseDoBot-pose))) > deg2rad(1)
        poseDoBot = poseDoBotFunct(q(i,:));
        DoBot.animate(poseDoBot);
        drawnow()
        pose = poseDoBotFunct(q(i,:));
    elseif rad2deg(max(abs(poseDoBot-pose))) < deg2rad(1)
        count = count + 1;
        count;
    end
%     UR3.animate(q(i,:));
%     drawnow()
end

end

function [output] = poseDoBotFunct(jointAngles)
    
    j1 = jointAngles(1);
    j2 = jointAngles(2);
    j3 = jointAngles(3);
    j4 = ((2*pi)-(j2 + pi)-(j3 + (pi/2))-(pi/2));
    j5 = jointAngles(4);
    
    output = [j1,j2,j3,j4,j5];
    
end
