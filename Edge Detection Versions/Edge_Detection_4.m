clear all

Image = imread('P1130326.jpg');
Original_Image = Image;
size = size(Image);

BW1 = rgb2gray(Image);
ED5 = edge(BW1,'prewitt',0.4);
ED6 = edge(BW1,'sobel',0.03);
ED7 = edge(BW1,'canny',0.16);
ED8 = edge(BW1,'roberts',0.03);

for i = 1 : size(1)
    for j = 1 : size(2)
        x = 0;
        if Image(i,j,1) >= 0 && Image(i,j,1) <= 45
            if Image(i,j,2) >= 0 && Image(i,j,2) <= 45
                if Image(i,j,3) >= 0 && Image(i,j,3) <= 100
                    Image(i,j,1) = 0;
                    Image(i,j,2) = 0;
                    Image(i,j,3) = 0;
                    x = 1;
                end
            end
        end
        if x == 0
            Image(i,j,1) = 225;
            Image(i,j,2) = 225;
            Image(i,j,3) = 225;
        end
    end
end

BW2 = rgb2gray(Image);
ED1 = edge(BW2,'prewitt',0);
ED2 = edge(BW2,'sobel',0);
ED3 = edge(BW2,'canny',0);
ED4 = edge(BW2,'roberts',0);

Image2 = zeros(size);

Images = {ED1 ED2 ED3 ED4 ED5 ED6 ED7 ED8};

for k = 1:8
    I = Images{k};
    for i = 1 : size(1)
        for j = 1 : size(2)
            if I(i,j,1) == 255 && I(i,j,2) == 255 && I(i,j,3) == 255
                    Image2(i,j,1) = 255;
                    Image2(i,j,2) = 255;
                    Image2(i,j,3) = 255;

            end
        end
    end
end


figure(1);
imshowpair(Original_Image,Image2,'montage')
title('Image                                      Edge Detection Filter');

% cornerPoints = detectHarrisFeatures(ED4, 'MinQuality', 0.5);
% 
% hold on
% plot(cornerPoints)