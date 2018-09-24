%% Sensors and Control for Mechatronics Systems
% Tutorial 5
% Feature Detection and Tracking
%% Part 1. Harris Corner Detection

I = imread('P1130325.jpg');
% figure(1);
% imshow(I);

I = rgb2gray(I);
% cornerPoints = detectHarrisFeatures(I, 'MinQuality', 0.2);

% imshow(I)
% hold on
% plot(cornerPoints)

% BW1 = edge(I,'sobel',0.03);
% BW2 = edge(I,'canny',0.16);
% BW3 = edge(I,'roberts',0.03);
BW4 = edge(I,'prewitt',0.03); %%Best

imshow(BW4);

% figure(2);
% imshowpair(BW1,BW2,'montage')
% title('Sobel Filter                                   Canny Filter');
% figure(3);
% imshowpair(BW3,BW4,'montage')
% title('Roberts Filter                                 Prewitt Filter');
% subplot('Position', [0.00 0.35 0.25 0.25])
% imshow(BW1)
% subplot('Position', [0.25 0.35 0.25 0.25])
% imshow(BW2)
% subplot('Position', [0.50 0.35 0.25 0.25])
% imshow(BW3)
% subplot('Position', [0.75 0.35 0.25 0.25])
% imshow(BW4)

% %% 1.1
% 
% clear all
% clc
% clf
% 
% I = imread('checkerboard.jpg');
% I = rgb2gray(I);
% cornerPoints = detectHarrisFeatures(I);
% 
% imshow(I)
% hold on
% plot(cornerPoints)
% 
% %% 1.(2-3)
% 
% clear all
% clc
% clf
% 
% I = imread('harris_corners_example.jpg');
% I = rgb2gray(I);
% cornerPoints = detectHarrisFeatures(I);
% 
% figure(1)
% imshow(I)
% hold on
% plot(cornerPoints)
% 
% %% 1.4
% 
% clear all
% clc
% % clf
% 
% I = imread('harris_corners_example.jpg');
% I = rgb2gray(I);
% cornerPoints = detectHarrisFeatures(I, 'MinQuality', 0.5);
% 
% figure(2)
% imshow(I)
% hold on
% plot(cornerPoints)
% 
% %% Part 2. SIFT Feature Extraction and Tracking
% 
% %% 2.1
% 
% run('C:\Users\admin\Documents\Jordans_Documents\UTS_SPR2018\41014_SCMS_-_Sensors_and_Control_for_Mechatronic_Systems\vlfeat-0.9.21\toolbox\vl_setup')
% vl_version verbose
% vl_setup demo
% vl_demo_sift_basic
% 
% %% 2.2
% 
% I1rgb = imread('roofs1.jpg');
% I2rgb = imread('roofs2.jpg');
% I1gs = rgb2gray(I1rgb);
% I2gs = rgb2gray(I2rgb);
% I1single = single(I1gs);
% I2single = single(I2gs);
% 
% %% 2.3
% 
% [f1, d1] = vl_sift(I1single);
% 
% imshow(I1gs);
% hold on
% h1 = vl_plotframe(f1);
% h2 = vl_plotframe(f1);
% set(h1,'color','k','linewidth',3) ;
% set(h1,'color','y','linewidth',2) ;
% 
% %% 2.4
% 
% [f2, d2] = vl_sift(I2single);
% [matches, scores] = vl_ubcmatch(d1, d2);
% 
% %% 2.5
% 
% showMatchedFeatures(I1gs, I2gs, f1(1:2, matches(1,:))', f2(1:2, matches(2,:))', 'montage');
% 
% %% Part 3. SURF Feature Extraction and Matching
% 
% %% 3.1
% 
% points1 = detectSURFFeatures(I1gs);
% points2 = detectSURFFeatures(I2gs);
% [features1, validPoints1] = extractFeatures(I1gs, points1);
% [features2, validPoints2] = extractFeatures(I2gs, points2);
% 
% %% 3.2
% 
% indexPairs = matchFeatures(features1, features2);
% matchedPoints1 = validPoints1(indexPairs(:,1));
% matchedPoints2 = validPoints2(indexPairs(:,2));
% 
% %% 3.3
% 
% showMatchedFeatures(I1gs, I2gs, matchedPoints1, matchedPoints2, 'montage')
% 
% %% 3.4
% 
% % Compare the results with the results you obtained in 2.5.
% 
% %% Part 4. RANSAC Outlier Rejection
% 
% %% 4.1
% 
% original = rgb2gray(imread('kfc1.jpg'));
% distorted = rgb2gray(imread('kfc2.jpg'));
% 
% %% 4.2
% 
% ptsOriginal = detectSURFFeatures(original);
% ptsDistorted = detectSURFFeatures(distorted);
% [featuresOriginal, validPtsOriginal] = extractFeatures(original, ptsOriginal);
% [featuresDistorted, validPtsDistorted] = extractFeatures(distorted, ptsDistorted);
% 
% %% 4.3
% 
% indexPairs = matchFeatures(featuresOriginal, featuresDistorted);
% matchedOriginal = validPtsOriginal(indexPairs(:,1));
% matchedDistorted = validPtsDistorted(indexPairs(:,2));
% 
% %% 4.4
% 
% figure;
% showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted);
% 
% %% 4.5
% 
% [tform, inlierDistorted, inlierOriginal] = estimateGeometricTransform(matchedDistorted,matchedOriginal, 'similarity');
% 
% %% 4.6
% 
% figure;
% showMatchedFeatures(original,distorted,inlierOriginal,inlierDistorted);
% title('Matching points (inliers only)');
% legend('ptsOriginal','ptsDistorted');
% 
% %% 4.7
% 
% Tinv = tform.invert.T;
% ss = Tinv(2,1);
% sc = Tinv(1,1);
% scaleRecovered = sqrt(ss*ss + sc*sc)
% thetaRecovered = atan2(ss,sc)*180/pi
% 
% %% 4.8
% 
% outputView = imref2d(size(original));
% recovered = imwarp(distorted,tform,'OutputView',outputView);
% figure, imshowpair(original,recovered,'montage')
