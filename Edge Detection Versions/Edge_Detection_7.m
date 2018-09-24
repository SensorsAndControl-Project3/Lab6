clear all

Image = imread('P1130324.jpg');
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

% array of cells to check
checkCells = false(size(1),size(2));

counter = 0;
checkCellsArray = {};

for i = 1 : (size(1)-1) : size(1)
    for j = 1 : size(2)
        if Image(i,j,1) == 255 && Image(i,j,2) == 255 && Image(i,j,3) == 255
            
            Image(i,j,1) = 0;
            Image(i,j,2) = 0;
            Image(i,j,3) = 0;
            
            if (i > 1)
                if (j > 1)
                    checkCells(i-1,j-1) = true;
                end
                checkCells(i-1,j) = true;
                if (j < size(2))
                    checkCells(i-1,j+1) = true;
                end
            end
            
            if (j > 1)
                checkCells(i,j-1) = true;
            end
            if (j < size(2))
                checkCells(i,j+1) = true;
            end
            
            if (i < size(1))
                if (j > 1)
                    checkCells(i+1,j-1) = true;
                end
                checkCells(i+1,j) = true;
                if (j < size(2))
                    checkCells(i+1,j+1) = true;
                end
            end
        end
    end
end

for j = 1 : (size(2)-1) : size(2)
    for i = 2 : (size(1)-1)
        if Image(i,j,1) == 255 && Image(i,j,2) == 255 && Image(i,j,3) == 255
            
            Image(i,j,1) = 0;
            Image(i,j,2) = 0;
            Image(i,j,3) = 0;
            
            if (i > 1)
                if (j > 1)
                    checkCells(i-1,j-1) = true;
                end
                checkCells(i-1,j) = true;
                if (j < size(2))
                    checkCells(i-1,j+1) = true;
                end
            end
            
            if (j > 1)
                checkCells(i,j-1) = true;
            end
            if (j < size(2))
                checkCells(i,j+1) = true;
            end
            
            if (i < size(1))
                if (j > 1)
                    checkCells(i+1,j-1) = true;
                end
                checkCells(i+1,j) = true;
                if (j < size(2))
                    checkCells(i+1,j+1) = true;
                end
            end
        end
    end
end

for i = 1 : size(1)
    for j = 1 : size(2)
        if checkCells (i,j) == true
            checkCellsArray = {checkCellsArray{:},[i,j]};
            counter = counter + 1;
        end
    end
end

checkCells = false(size(1),size(2));

count = 0;

while counter > 0
    newcheckCellsArray = {};
    newcounter = 0;
    forcount = 0;
    x = counter;
    counter = 0;
    for k = 1 : x
        cell = checkCellsArray{k};
        i = cell(1);
        j = cell(2);
        
%         newcheckCellsArray = {};
%         for h = 2 : counter
%             newcheckCellsArray = {newcheckCellsArray{:},checkCellsArray{h}};
%         end
%         counter = counter - 1;
%         checkCellsArray = {};
%         for g = 1 : counter
%             checkCellsArray = {checkCellsArray{:},newcheckCellsArray{1}};
%         end

        if Image(i,j,1) == 255 && Image(i,j,2) == 255 && Image(i,j,3) == 255

            Image(i,j,1) = 0;
            Image(i,j,2) = 0;
            Image(i,j,3) = 0;

            if (i > 1)
                if (j > 1)
%                     checkCells(i-1,j-1) = true;
                    [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i-1,j-1,Image);
                end
%                 checkCells(i-1,j) = true;
                [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i-1,j,Image);
                if (j < size(2))
%                     checkCells(i-1,j+1) = true;
                    [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i-1,j+1,Image);
                end
            end

            if (j > 1)
%                 checkCells(i,j-1) = true;
                [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i,j-1,Image);
            end
            if (j < size(2))
%                 checkCells(i,j+1) = true;
                [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i,j+1,Image);
            end

            if (i < size(1))
                if (j > 1)
%                     checkCells(i+1,j-1) = true;
                    [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i+1,j-1,Image);
                end
%                 checkCells(i+1,j) = true;
                [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i+1,j,Image);
                if (j < size(2))
%                     checkCells(i+1,j+1) = true;
                    [newcheckCellsArray,newcounter,Image] = add2Array(newcheckCellsArray,newcounter,i+1,j+1,Image);
                end
            end
        end
        forcount = forcount + 1;
        disp(forcount);
    end
    
    checkCellsArray = {newcheckCellsArray{:}};
    counter = newcounter;
    
    count = count + 1;
    disp(count);
    figure(2);
    imshow(Image);
    
end

BW = rgb2gray(Image);
% Uncomment if you would like to detect the edges
% ED = edge(BW,'roberts',0);

figure(1);
imshowpair(Original_Image,Image,'montage')
title('Image                                      Object Detection Filter');

%% making a true/false black and white image

imwrite(Image, 'image.png');
Processed_Image = imread('image.png');
BW0 = rgb2gray(Processed_Image);
BW1 = false(size(1),size(2));

for i = 1:size(1)
    for j = 1:size(2)
        
        if BW0(i,j) == 255
            BW1(i,j) = true;
        end
        
    end
end


%% Look for the 4 largest white shapes

BW2 = BW1;

Add = false(size(1),size(2));

for x = 1:4
    BW3 = bwpropfilt(BW2,'area',1);
    figure;
    imshow(BW3)
    title('Objects with the Largest Perimeters')
    
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


function [array,num,Image] = add2Array(array,num,i,j,Image)
    if Image(i,j,1) == 255 && Image(i,j,2) == 255 && Image(i,j,3) == 255
        includedInArray = false;
        for c = 1 : num
            if array{num} == [i,j]
                includedInArray = true;
                c = num;
            end
        end

        if includedInArray == false
            array = {array{:},[i,j]};
            num = num + 1;
        end
    end
end