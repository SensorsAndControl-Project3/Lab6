%%  set up
clear

%Focal Length:          
fc = [29092.73970,21390.43007];
%Principal point:       
cc = [2079.50000,1559.50000];

I1 = imread('1.jpg');
I1 = rgb2gray(I1);
I2 = imread('2.jpg');
I2 = rgb2gray(I2);


%%  image one left
points = detectHarrisFeatures(I1)
size = 25;
% imshow(I1); hold on
% plot(points.selectStrongest(size))

P = points.selectStrongest(size)

locations = P.Location();

% xz_left = [int16(locations(23,1)), int16(locations(23,2))];
xz_left = [locations(23,1), locations(23,2)];
xz_left

sq = 20;
for i = 1 : sq
    for j = 1 : sq
        %b = b + [i,j]
        c = xz_left(2) + i - sq/2;
        d = xz_left(1) + j - sq/2;
        [c,d];
        %c = int16(c); d = int16(d);
        I1(int16(c),int16(d)) = 0;
        %I1(c,d,2) = 0;
    end
end

%%  image two right
points = detectHarrisFeatures(I2)
size = 66;
% imshow(I2); hold on
% plot(points.selectStrongest(size))

P = points.selectStrongest(size)

locations = P.Location();

% xz_right = [int16(locations(66,1)), int16(locations(66,2))];
xz_right = [locations(66,1), locations(66,2)];
xz_right

sq = 20;
for i = 1 : sq
    for j = 1 : sq
        %b = b + [i,j]
        c = xz_right(2) + i - sq/2;
        d = xz_right(1) + j - sq/2;
        [c,d];
        %c = int16(c); d = int16(d);
        I2(int16(c),int16(d)) = 0;
        %I1(c,d,2) = 0;
    end
end

%%  show images
% figure(1)
% imshow(I1)
% figure(2)
% imshow(I2)

%% finding distance away

focal = fc;
fx = focal(1)
fz = focal(2)
PriciplePoint = cc;
PPx = PriciplePoint(1)
PPz = PriciplePoint(2)
xz_left
xz_right
B = 50*10^-3
dx = xz_right(1) - xz_left(1)
dz = xz_right(2) - xz_left(2)

Z = (B * fx) / (dx * 10 )
X = (Z * xz_right(1)* 10) / (fx)