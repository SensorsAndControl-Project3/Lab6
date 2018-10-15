% DoBot controller for controlling the real DoBot
clf % Clear Figure
Initialise = 1;
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
    pause(1)
    state_msg.JointAngles
    
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
    joint_msg.JointAngles(2) = 0.7857;
    joint_msg.JointAngles(3) = 0.7863;
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

pause(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% read from the camera for the first image

figure
imageSub = rossubscriber('/camera/color/image_raw');
pause(1);
image1 = readImage(imageSub.LatestMessage);
imshow(image1);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% move over the right side to position 2 to take scan
joint_msg.JointAngles(1) = pi/2;
joint_msg.JointAngles(2) = 0.1772
% joint_msg.JointAngles(2) = deg2rad(10.1528);
joint_msg.JointAngles(3) = -0.0069
% joint_msg.JointAngles(2) = deg2rad(-0.3953);
joint_msg.JointAngles(4) = 0;
joint_srv.call(joint_msg);

pause(2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% read from the camera for the second image

figure
imageSub = rossubscriber('/camera/color/image_raw');
pause(1);
image2 = readImage(imageSub.LatestMessage);
imshow(image2);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% calculate the X and Y values of each shape

CoordArray = intelCamera(image1, image2);

ScatteredPose1 = [CoordArray(1,1),CoordArray(1,2)];
ScatteredPose2 = [CoordArray(2,1),CoordArray(2,2)];
ScatteredPose3 = [CoordArray(3,1),CoordArray(3,2)];
ScatteredPose4 = [CoordArray(4,1),CoordArray(4,2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% these will become the inputs from the code above
% Scattered Shapes position [X,Y]
% ScatteredPose1 = [(-72+50),(318-80)];
% ScatteredPose2 = [(55+50),(318-80)];
% ScatteredPose3 = [(72+50),(267-80)];
% ScatteredPose4 = [(-60+50),(255-80)];

% ScatteredPose1 = [(-72+30+100),(318-80)];
% ScatteredPose2 = [(55+100),(318-80)];
% ScatteredPose3 = [(72+100),(267-80)];
% ScatteredPose4 = [(-60+30+100),(255-80)];

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
SortedPose1 = [(6-50),(326-80)];
SortedPose2 = [(2-50),(242-80)];
SortedPose3 = [(-54-50),(339-80)];
SortedPose4 = [(69-50),(327-80)];

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

pickDepthZ1 = 250-95-50;
pickDepthZ2 = 250-95-85;
placeDepthZ = 220-90-50;

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
        
%         disp('paused - press any key to continue')
%         pause;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Offset of the camera with respect to the end effector
        % Camera2EndeffectorCoords = [X,Y,Z];
        Camera2EndeffectorCoords = [0,40/1000,45/1000];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        rotation = 0;
        
        if pickPlaceNum == 1 && sideNum == 1
            
            BoxCoords = [ScatteredPose1,pickDepthZ1];
            
        elseif pickPlaceNum == 1  && sideNum == 2
            
            BoxCoords = [SortedPose1,placeDepthZ];
            rotation = RotationAngle1;
            
        elseif pickPlaceNum == 2  && sideNum == 1
            
            BoxCoords = [ScatteredPose2,pickDepthZ1];
            
        elseif pickPlaceNum == 2  && sideNum == 2
            
            BoxCoords = [SortedPose2,placeDepthZ];
            rotation = RotationAngle2;
            
        elseif pickPlaceNum == 3 && sideNum == 1
            
            BoxCoords = [ScatteredPose3,pickDepthZ1];
            
        elseif pickPlaceNum == 3  && sideNum == 2
            
            BoxCoords = [SortedPose3,placeDepthZ];
            rotation = RotationAngle3;
            
        elseif pickPlaceNum == 4  && sideNum == 1
            
            BoxCoords = [ScatteredPose4,pickDepthZ1];
               
        elseif pickPlaceNum == 4  && sideNum == 2
            
            BoxCoords = [SortedPose4,placeDepthZ];
            rotation = RotationAngle4;
            
        end
        
        BoxCoords = BoxCoords/1000; % box pick or box position to place
        BoxCoordinates = BoxCoords + Camera2EndeffectorCoords;
        AboveBoxCoordinates = [BoxCoordinates(1), BoxCoordinates(2), 50/1000];
        
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
L12 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([-10 100]),'offset',pi/2);
L13 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-55 55]),'offset',-pi/2);
L14 = Link('d',0,'a',-wristDist,'alpha',pi/2,'qlim',deg2rad([-100 100]),'offset',0);
L15 = Link('d',endEffectorDist,'a',0,'alpha',0,'qlim',deg2rad([-105 105]),'offset',0);

% model for ikine and ikcon
A = wristDist; % Wrist: 61.5 mm
B = endEffectorDist; % End Effector: 79.5 mm
C = sqrt((A^2)+(B^2));
theta = atan(B/A);
L21 = Link('d',baseDist,'a',0,'alpha',pi/2,'qlim',deg2rad([-135 135]),'offset',0);
L22 = Link('d',0,'a',lowerArmDist,'alpha',0,'qlim',deg2rad([-10 100]),'offset',pi/2);
L23 = Link('d',0,'a',-upperArmDist,'alpha',0,'qlim',deg2rad([-55 55]),'offset',-pi/2);
L24 = Link('d',0,'a',-C,'alpha',0,'qlim',deg2rad([-100 100]),'offset',theta);

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
xArm = ImageEndEffectorPose(1,4);
yArm = ImageEndEffectorPose(2,4);
Base2EndEffector = [ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)';ImageEndEffectorPose(1:3,4)'];
if PalletSidefn == 0
    
    Camera2Object = [0,0,0;
        -AboveBoxCoordinates(1),-AboveBoxCoordinates(2),-AboveBoxCoordinates(3);
        -BoxCoordinates(1), -BoxCoordinates(2), -BoxCoordinates(3);
        -AboveBoxCoordinates(1),-AboveBoxCoordinates(2),-AboveBoxCoordinates(3);
        0,0,0];
    
%     Camera2Object = [0,0,0;
%         -AboveBoxCoordinates(1)-xArm,-AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
%         -BoxCoordinates(1)-xArm, -BoxCoordinates(2)-yArm, -BoxCoordinates(3);
%         -AboveBoxCoordinates(1)-xArm,-AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
%         0,0,0];
    
elseif PalletSidefn == 1
    
    Camera2Object = [0,0,0;
        AboveBoxCoordinates(1)-xArm,AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
        BoxCoordinates(1)-xArm, BoxCoordinates(2)-yArm, -BoxCoordinates(3);
        AboveBoxCoordinates(1)-xArm,AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
        0,0,0];
    
%     Camera2Object = [0,0,0;
%         AboveBoxCoordinates(1)-xArm,AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
%         BoxCoordinates(1)-xArm, BoxCoordinates(2)-yArm, -BoxCoordinates(3);
%         AboveBoxCoordinates(1)-xArm,AboveBoxCoordinates(2)-yArm,-AboveBoxCoordinates(3);
%         0,0,0];
    
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
joint_msg.JointAngles(4) = -deg2rad(rotation);
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
    By = 20;
    dx = xy_left(1) - xy_right(1);
    dy = xy_left(2) - xy_right(2);
    %from the left image
    
    Z = (By * fy) / (dy);
    X = (Z * xy_left(1)) / (fx);
    Y = (Z * xy_left(2)) / (fy);
    
    X = -round(X);
    Y = round(Y);
    Z = round(Z);
    
    ShapeCoordinates(c1,1) = X;
    ShapeCoordinates(c1,2) = Y;
    ShapeCoordinates(c1,3) = Z;
    
    display(['Distance to Shape: ',num2str(c1), ' is (X = ', num2str(X), ',Y = ',num2str(Y), ',Z = ',num2str(Z), ')'])
    
end

CoordArray = ShapeCoordinates;

end
