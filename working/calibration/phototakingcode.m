% DoBot controller for controlling the real DoBot

% if 1 run initialisation
% else don't run initialisation
Initialise = 1
clf
if Initialise == 1
    close all
    clear all
%     set(0,'DefaultFigureWindowStyle','docked')
    clc
    
    try rosshutdown
    end
    
    ipaddress = '192.168.31.141';
    rosinit(ipaddress)
    rostopic list
    rosservice list
end
for i = 1:100
%     figure(2)
    imageSub = rossubscriber('/camera/color/image_raw');
    pause(1);
    image = readImage(imageSub.LatestMessage);
%     image_h = imshow(readImage(imageSub.LatestMessage));
    filename = num2str(i);
    filename = '1'
    filename = strcat(filename,'.jpg')
    imwrite(image, filename);
    pause(3);
end