% DoBot controller for controlling the real DoBot

% if 1 run initialisation
% else don't run initialisation
Initialise = 1
clf
if Initialise == 1
    clf
    close all
    clear all
    set(0,'DefaultFigureWindowStyle','docked')
    clc
    
    try rosshutdown
    end
    
    ipaddress = '192.168.31.141';
    rosinit(ipaddress)
    rostopic list
    rosservice list
    
    % initialise Image Subscriber
    imageSub = rossubscriber('/camera/color/image_raw');
    
    % Gives joint angles
    pause(1)
    dobot_state_sub = rossubscriber('/dobot_magician/state');
    pause(1)
    receive(dobot_state_sub,2);
    state_msg = dobot_state_sub.LatestMessage;
    state_msg.JointAngles;
   
    % sets up the ros client
    pause(1)
    joint_srv = rossvcclient('/dobot_magician/joint_angs');
    joint_msg = rosmessage(joint_srv)
    joint_msg.JointAngles
    
    % dobot_magiciam pump control
    pause(1)
    pump_srv = rossvcclient('/dobot_magician/pump');
    pump_msg = rosmessage(pump_srv);
    pump_msg.Pump = 0;
%     pause(1)
    try pump_srv.call(pump_msg); end
    
    % Initialise Pose
    joint_msg.JointAngles(1) = 0;
    joint_msg.JointAngles(2) = 0;
    joint_msg.JointAngles(3) = 0;
    joint_msg.JointAngles(4) = 0;
    joint_srv.call(joint_msg);
    
end
pause(1);
joint_msg.JointAngles(1) = 0.05;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

leftImageJointAngles = [0.05,0,0,0,0];

% move the Dobot for the Left image
% Dobot();

figure
imageSub = rossubscriber('/camera/color/image_raw');
pause(1);
image1 = readImage(imageSub.LatestMessage);
imshow(image1);

joint_msg.JointAngles(1) = -0.05;
joint_msg.JointAngles(2) = 0;
joint_msg.JointAngles(3) = 0;
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

% move the Dobot for the right image
% Dobot();

figure
imageSub = rossubscriber('/camera/color/image_raw');
pause(1);
image2 = readImage(imageSub.LatestMessage);
imshow(image2);

CoordArray = intelCamera(image1, image2);

Dobot(CoordArray, leftImageJointAngles);

function Dobot(CoordArray, leftImageJointAngles)

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

% Plotting the Visual Model
figure
DoBotVisuModel.plot(zeros(1,DoBotVisuModel.n),'workspace',workspace,'scale',scale, 'noarrow');
hold on
DoBotVisuModel.teach
% Specifying the coordinates that the robot arm need to move to
leftImageEndEffectorPose = DoBotVisuModel.fkine(leftImageJointAngles)

Base2EndEffector = [leftImageEndEffectorPose(1:3,4)';leftImageEndEffectorPose(1:3,4)';leftImageEndEffectorPose(1:3,4)';leftImageEndEffectorPose(1:3,4)']
EndEffector2Camera = [-0.022,0.000,0.055;-0.022,0.000,0.055;-0.022,0.000,0.055;-0.022,0.000,0.055]
Camera2Object = [CoordArray(:,2)/1000,CoordArray(:,1)/1000,CoordArray(:,3)/1000]

shapeCoordinates = (Base2EndEffector + EndEffector2Camera + Camera2Object)
zeroCoordinates = leftImageEndEffectorPose(1:3,4)';
Coordinates = [shapeCoordinates(1,:); zeroCoordinates; shapeCoordinates(2,:); zeroCoordinates; shapeCoordinates(3,:); zeroCoordinates; shapeCoordinates(4,:); zeroCoordinates; zeroCoordinates]
PumpOperation = [0,1,0,1,0,1,0,1,0];

% Creating a loop depending on the number of coordinates that the robot
% must move to as specified above
sizeof = size(Coordinates);
for x = 1:sizeof(1)
    
    pump(PumpOperation(x));
    
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

function [CoordArray] = intelCamera(image1, image2)

%% best code

Centroids = zeros(8,2);

ImageArray = {image1,image2};
sizeArray = size(ImageArray);
for imageNumber = 1:(sizeArray(2))
    Image = ImageArray{imageNumber};
    Original_Image = Image;
    try clear size; end
    sizeA = size(Image);
    
    for i = 1 : sizeA(1)
        for j = 1 : sizeA(2)
            x = 0;
            if Image(i,j,1) >= 0 && Image(i,j,1) <= 80
                if Image(i,j,2) >= 0 && Image(i,j,2) <= 80
                    if Image(i,j,3) >= 0 && Image(i,j,3) <= 180
                        Image(i,j,1) = 255;
                        Image(i,j,2) = 255;
                        Image(i,j,3) = 255;
                        x = 1;
                    end
                end
            end
            if x == 0
                Image(i,j,1) = 0;
                Image(i,j,2) = 0;
                Image(i,j,3) = 0;
            end
        end
    end
    imwrite(Image, 'image.jpg');
    Image = imread('image.jpg');
    BW = rgb2gray(Image);
    ED = edge(BW,'roberts',0);
    
    figure;
    imshowpair(Original_Image,Image,'montage')
    title('Image                                      Edge Detection Filter');
    
    BW0 = rgb2gray(Image);
    BW1 = false(sizeA(1),sizeA(2));
    
    for i = 1:sizeA(1)
        for j = 1:sizeA(2)
            
            if BW0(i,j) == 255
                BW1(i,j) = true;
            end
            
        end
    end
    
    BW2 = BW1;
    
    Add = false(sizeA(1),sizeA(2));
    
    for x = 1:4
        BW3 = bwpropfilt(BW2,'area',1);
        figure;
        imshow(BW3)
        
        if imageNumber == 1
            u = x;
        elseif imageNumber == 2
            u = x+4;
        end
        
        title(['Shape: ', num2str(u)])
        
        hold on
        s = regionprops(BW3,'centroid');
        centroids = cat(1, s.Centroid);
        plot(centroids(:,1),centroids(:,2), 'b*')
        
        st = regionprops(BW3, 'BoundingBox', 'Area' );
        [maxArea, indexOfMax] = max([st.Area]);
        rectangle('Position',[st(indexOfMax).BoundingBox(1),st(indexOfMax).BoundingBox(2),st(indexOfMax).BoundingBox(3),st(indexOfMax).BoundingBox(4)], 'EdgeColor','r','LineWidth',2 )
        
        st = regionprops(BW3,'Orientation','MajorAxisLength');
        a = s.Centroid(1) + st.MajorAxisLength * cosd(st.Orientation);
        b = s.Centroid(2) - st.MajorAxisLength * sind(st.Orientation);
        %     line([c(1) a],[c(2) b]);
        
        st = regionprops(BW3,'Extrema');
        line([st.Extrema(7),st.Extrema(4)],[st.Extrema(15),st.Extrema(12)], 'Color', 'red', 'Linestyle', '--');
        plot(st.Extrema(4),st.Extrema(12), 'r*')
        plot(st.Extrema(7),st.Extrema(15), 'g*')
        
        hold off
        if imageNumber == 1
            Centroids(x,1) = round(centroids(:,1));
            Centroids(x,2) = round(centroids(:,2));
        elseif imageNumber == 2
            centroidsNumber = x+4;
            Centroids(centroidsNumber,1) = round(centroids(:,1));
            Centroids(centroidsNumber,2) = round(centroids(:,2));
        end
        
        for i = 1:sizeA(1)
            for j = 1:sizeA(2)
                if BW3(i,j) == true
                    Add(i,j) = true;
                end
            end
        end
        
        BW2 = false(sizeA(1),sizeA(2));
        for i = 1:sizeA(1)
            for j = 1:sizeA(2)
                if BW1(i,j) == true && Add(i,j) == false
                    BW2(i,j) = true;
                end
            end
        end
    end
end

for x=1:8
    display(['Centre of Shape: ',num2str(x), ' is (', num2str(Centroids(x,1)), ',',num2str(Centroids(x,2)), ')'])
end

ShapeCoordinates = zeros(3,4);

%% finding distance away
for c1 = 1:4
    xy_left = [Centroids(c1,1),Centroids(c1,2)];
    c2 = c1+4;
    xy_right = [Centroids(c2,1),Centroids(c2,2)];
    
    % before auto calib
    % Focal Length:
    fc = [ 920.2548   923.2314 ];
    % Principal point:
    cc = [ 319.50000   239.50000 ];
    
    
    focal = fc;
    fx = focal(1);
    fy = focal(2);
    PriciplePoint = cc;
    xy_left = xy_left - PriciplePoint;
    xy_right = xy_right - PriciplePoint;
    Bx = 25;
    dx = xy_left(1) - xy_right(1);
    dz = xy_left(2) - xy_right(2);
    %from the left image
    
    Z = (Bx * fx) / (dx);
    X = (Z * xy_left(1)) / (fx);
    Y = (Z * xy_left(2)) / (fy);
    
    X = round(X);
    Y = round(Y);
    Z = round(Z);
    
    ShapeCoordinates(c1,1) = X;
    ShapeCoordinates(c1,2) = Y;
    ShapeCoordinates(c1,3) = Z;
    
    display(['Distance to Shape: ',num2str(c1), ' is (X = ', num2str(X), ',Y = ',num2str(Y), ',Z = ',num2str(Z), ')'])
    
end

CoordArray = ShapeCoordinates;

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

