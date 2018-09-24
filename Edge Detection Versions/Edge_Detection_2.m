clear all

Image = imread('P1130326.jpg');

Original_Image = Image;

% BW = rgb2gray(Image);

% ED = edge(BW,'prewitt',0.03); %%Best

size = size(Image)

size(1)
size(2)

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

Image2 = zeros(size);

for i = 1 : size(1)

    for j = 1 : size(2)
        
        x = 0;
        
        if Image(i,j,1) == 0 && Image(i,j,2) == 0 && Image(i,j,3) == 0
            
            if Image(i+1,j+1,1) == 0 && Image(i,j+1,2) == 0 && Image(i-1,j+1,3) == 0 ...
                    && Image(i+1,j+1,1) == 0 && Image(i-1,j,3) == 0 ...
                    && Image(i+1,j-1,1) == 0 && Image(i,j-1,2) == 0 && Image(i-1,j-1,3) == 0
                
%             if Image(i+2,j+2) == 0 && Image(i+1,j+2) == 0 && Image(i,j+2) == 0 && Image(i-1,j+2) == 0 && Image(i-2,j+2) == 0 ...
%                     && Image(i+2,j+1) == 0 && Image(i+1,j+1) == 0 && Image(i,j+1) == 0 && Image(i-1,j+1) == 0 && Image(i-2,j+1) == 0 ...
%                     && Image(i+2,j) == 0 && Image(i+1,j) == 0 && Image(i,j) == 0 && Image(i-1,j) == 0 && Image(i-2,j) == 0 ...
%                     && Image(i+2,j-1) == 0 && Image(i+1,j-1) == 0 && Image(i,j-1) == 0 && Image(i-1,j-1) == 0 && Image(i-2,j-1) == 0 ...
%                     && Image(i+2,j-2) == 0 && Image(i+1,j-2) == 0 && Image(i,j-2) == 0 && Image(i-1,j-2) == 0 && Image(i-2,j-2) == 0 ...

                Image2(i,j,1) = 255;
                Image2(i,j,2) = 255;
                Image2(i,j,3) = 255;
            else
                
            end
        end
    
    end
    
end

BW = rgb2gray(Image2);
% ED1 = edge(BW,'prewitt',0.4); %%Best
% ED2 = edge(BW,'sobel',0.03);
% ED3 = edge(BW,'canny',0.16);
% ED4 = edge(BW,'roberts',0.03);

% ED1 = edge(BW,'prewitt',0); %%Best
% ED2 = edge(BW,'sobel',0);
% ED3 = edge(BW,'canny',0);
ED4 = edge(BW,'roberts',0);

figure(1);
imshowpair(Original_Image,ED4,'montage')
title('Image                                      Edge Detection Filter');

% figure(1);
% imshow(ED1)
% 
% figure(2);
% imshow(ED2)
% 
% figure(3);
% imshow(ED3)
% 
% figure(4);
% imshow(ED4)