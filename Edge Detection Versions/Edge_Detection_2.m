clear all

Image = imread('P1130326.jpg');

Original_Image = Image;

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
                
                Image2(i,j,1) = 255;
                Image2(i,j,2) = 255;
                Image2(i,j,3) = 255;
            else
                
            end
        end
    
    end
    
end

BW = rgb2gray(Image2);

ED4 = edge(BW,'roberts',0);

figure(1);
imshowpair(Original_Image,ED4,'montage')
title('Image                                      Edge Detection Filter');