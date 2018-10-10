%% new code for shape detection added in centre of shape
clc
clear all
close all
clear
%% best code

Image1 = imread('P1130323.jpg');
Image2 = imread('P1130325.jpg');

Centroids = zeros(8,2);

ImageArray = {Image1,Image2};
sizeArray = size(ImageArray);
for imageNumber = 1:(sizeArray(2))
    Image = ImageArray{imageNumber};
    Original_Image = Image;
    try clear size; end
    size = size(Image);
    
    for i = 1 : size(1)
        for j = 1 : size(2)
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
    
    BW = rgb2gray(Image);
    ED = edge(BW,'roberts',0);
    
    figure;
    imshowpair(Original_Image,Image,'montage')
    title('Image                                      Edge Detection Filter');
    
    BW0 = rgb2gray(Image);
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
Bx = 20;
dx = xy_left(1) - xy_right(1);
dz = xy_left(2) - xy_right(2);
%from the left image

Z = (Bx * fx) / (dx);
X = (Z * xy_left(1)) / (fx);
Y = (Z * xy_left(2)) / (fy);

X = round(X);
Y = round(Y);
Z = round(Z);

ShapeCoordinates(1,c1) = X;
ShapeCoordinates(2,c1) = Y;
ShapeCoordinates(3,c1) = Z;

display(['Distance to Shape: ',num2str(c1), ' is (x = ', num2str(X), ',y = ',num2str(Y), ',Z = ',num2str(Z), ')'])

end