clear
clc

% these will become the inputs
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
