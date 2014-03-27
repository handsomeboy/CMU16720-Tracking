function [ moving_image ] = SubtractDominantMotion2( image1, image2 )
% Estimate the dominant motion
% Inputs:
% image1, image2 - two frames of images
% Outputs:
% moving_image - a binary image of in which non-zero pixels correspond to
% locations of moving objects

% first warp the images
M = LucasKanadeAffine(image1, image2);
warpimg = warpH(image1, M, [size(image2,1) size(image2,2)]);

% compute mask
[X, Y] = meshgrid(1:size(image1,2), 1:size(image2,1));
warpx = M(1,1)*X + M(1,2)*Y + M(1,3);
warpy = M(2,1)*X + M(2,2)*Y + M(2,3);
mask = zeros(size(image2,1), size(image2,2));
mask = mask | (warpx >=1 & warpx <= size(image2,2));
mask = mask & (warpy >=1 & warpy <= size(image2,1));

% compute difference between images
deltaI = abs(image2 - warpimg);
deltaI = deltaI/255;
deltaI = mask .* deltaI;

moving_image = medfilt2(hysthresh(deltaI, 0.23, 0.19));

se = strel('disk',8);
moving_image = imdilate(moving_image, se);
moving_image = imerode(moving_image, se);
moving_image = medfilt2(moving_image);
moving_image = double(moving_image);

% remove big patches
subimage = bwareaopen(moving_image, 100);
moving_image = moving_image - subimage;

% remove small patches
%moving_image = bwareaopen(moving_image, 8);

end
