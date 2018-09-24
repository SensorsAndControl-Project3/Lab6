I = imread('P1130325.jpg');

I = rgb2gray(I);

BW4 = edge(I,'prewitt',0.03);

imshow(BW4);
