%% new code for shape detection added in centre of shape
clc
clear all
close all
clear
%% best code

Image = imread('P1130323.jpg');
Original_Image = Image;
size = size(Image);

for i = 1 : size(1)
    for j = 1 : size(2)
        x = 0;
        if Image(i,j,1) >= 0 && Image(i,j,1) <= 45
            if Image(i,j,2) >= 0 && Image(i,j,2) <= 45
                if Image(i,j,3) >= 0 && Image(i,j,3) <= 90
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

BW = rgb2gray(Image);
ED = edge(BW,'roberts',0);

figure(1);
imshowpair(Original_Image,Image,'montage')
title('Image                                      Edge Detection Filter');

% imwrite(Image, 'image.png');
% Processed_Image = imread('image.png');
Processed_Image = Image;
BW0 = rgb2gray(Processed_Image);
% sizeImage = size(BW0)
BW1 = false(size(1),size(2));

for i = 1:size(1)
    for j = 1:size(2)
        
        if BW0(i,j) == 255
            BW1(i,j) = true;
        end
        
    end
end

BW2 = BW1;

Add = false(size(1),size(2));

Centroids = zeros(4,2);

for x = 1:4
    BW3 = bwpropfilt(BW2,'area',1);
    figure;
    imshow(BW3)
    title(['Shape: ', num2str(x)])
    hold on
    
    s = regionprops(BW3,'centroid');
    centroids = cat(1, s.Centroid);
    hold on
    plot(centroids(:,1),centroids(:,2), 'b*')
    hold off
    
    Centroids(x,1) = round(centroids(:,1));
    Centroids(x,2) = round(centroids(:,2));

    for i = 1:size(1)
        for j = 1:size(2)
            if BW3(i,j) == true
                Add(i,j) = true;
            end
        end
    end

    BW2 = false(size(1),size(2));
    for i = 1:size(1)
        for j = 1:size(2)
            if BW1(i,j) == true && Add(i,j) == false
                BW2(i,j) = true;
            end
        end
    end
end

for x=1:4
    display(['Centre of Shape: ',num2str(x), ' is (', num2str(Centroids(x,1)), ',',num2str(Centroids(x,2)), ')'])
end

% BW2 = bwpropfilt(BW1,'perimeter',1);
% figure;
% imshow(BW2)
% title('Objects with the Largest Area')


% BW2 = bwpropfilt(BW1,'EulerNumber',[1 1]);
% figure
% imshow(BW2)
% title('Regions with Euler Number == 1')