%% new code for shape detection added in centre of shape
clc
clear all
close all
clear
%% best code

Image1 = imread('angletest7.jpg');
Image2 = imread('angletest7.jpg');

Centroids = zeros(2,2);

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
    
    BW3 = bwpropfilt(BW2,'area',1);
    figure;
    imshow(BW3)
    
    title(['Shape: ', num2str(imageNumber)])
    
    hold on
    s = regionprops(BW3,'centroid');
    centroids = cat(1, s.Centroid);
    plot(centroids(:,1),centroids(:,2), 'b*')
    
    st = regionprops(BW3, 'BoundingBox', 'Area' );
    [maxArea, indexOfMax] = max([st.Area]);
    rectangle('Position',[st(indexOfMax).BoundingBox(1),st(indexOfMax).BoundingBox(2),st(indexOfMax).BoundingBox(3),st(indexOfMax).BoundingBox(4)], 'EdgeColor','r','LineWidth',2 )
    
    st = regionprops(BW3,'Orientation','MajorAxisLength');
    OrientationFromAboveAngle = st.Orientation;
    a = s.Centroid(1) + st.MajorAxisLength * cosd(st.Orientation);
    b = s.Centroid(2) - st.MajorAxisLength * sind(st.Orientation);
    
    st = regionprops(BW3,'Extrema');
    line([st.Extrema(7),st.Extrema(4)],[st.Extrema(15),st.Extrema(12)], 'Color', 'red', 'Linestyle', '--');
    plot(st.Extrema(4),st.Extrema(12), 'r*')
    plot(st.Extrema(7),st.Extrema(15), 'g*')
    point_right = [st.Extrema(4) , st.Extrema(12)];
    point_left = [st.Extrema(7) , st.Extrema(15)];
    
    hold off
    
    Centroids(imageNumber,1) = round(centroids(:,1));
    Centroids(imageNumber,2) = round(centroids(:,2));
end

for x=1:2
    display(['Centre of Shape: ',num2str(x), ' is (', num2str(Centroids(x,1)), ',',num2str(Centroids(x,2)), ')'])
end

ShapeCoordinates = zeros(3);

%% finding distance away

xy_left = [Centroids(1,1),Centroids(1,2)];
xy_right = [Centroids(2,1),Centroids(2,2)];

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

ShapeCoordinates(1) = X;
ShapeCoordinates(2) = Y;
ShapeCoordinates(3) = Z;

display(['Distance to Shape is (x = ', num2str(X), ',y = ',num2str(Y), ',Z = ',num2str(Z), ')'])

%%  finding angle
point_right;
point_left;

rise = point_left(2) - point_right(2);
run = point_right(1) - point_left(1);
angle = atand(rise/run);
%   this is the angle from finding the inverse tan between two points
if angle < 0
    string = 'depression';
    angle = -1 * angle;
else
    string = 'elevation';
    angle;
end
% angle = round(angle)
display(['this is the angle of ', string ,' form rise over run at angle = ', num2str(angle)])
%   this is the angle given from the code above
if OrientationFromAboveAngle < 0
    string = 'depression';
    OrientationFromAboveAngle = -1 * OrientationFromAboveAngle;
else
    string = 'elevation';
    OrientationFromAboveAngle;
end
display(['this is the angle of ', string ,'from the at.Orination at angle = ', num2str(OrientationFromAboveAngle)])

