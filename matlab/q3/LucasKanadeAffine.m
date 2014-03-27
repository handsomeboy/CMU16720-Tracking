function [ M ] = LucasKanadeAffine( It, It1 )
% Lucas Kanade tracker for affine motion
% Inputs:
% It - image frame at time t
% It1 - image frame at time t+1
% Outputs:
% M - affine transformation matrix

epsilon = 0.001;
deltaP = 1;
param = zeros(1,6);

%It = rgb2gray(It);
%It1 = rgb2gray(It1);
It = mat2gray(It);
It1 = mat2gray(It1);
template = It;

[X, Y] = meshgrid(1:size(It,2), 1:size(It,1));
Xpoints = X(:);
Ypoints = Y(:);
points = [Xpoints'; Ypoints'; ones(1, size(It,1)*size(It,2))];
size(points);

while norm(deltaP) > epsilon
    % first compute the warp image
    W = [1+param(1) param(3) param(5);
            param(2) 1+param(4) param(6)];
    warpx = W(1)*X + W(3)*Y + W(5);
    warpy = W(2)*X + W(4)*Y + W(6);
    
    size(warpx);
    
    warpimg = interp2(X, Y, It1, warpx, warpy);
    size(warpimg);
    warpimg(isnan(warpimg)) = 0;
    
    % find the common regions between warp image and It1
    mask = zeros(size(It,1), size(It,2));
    mask = mask | (warpx >=1 & warpx <= size(It,2));
    mask = mask & (warpy >=1 & warpy <= size(It,1));
    
    % compute error
    error = template - warpimg;
    error = mask .* error;
    error = error(:);

    % compute gradient
    [dIx, dIy] = gradient(warpimg);
    dIx = mask .* dIx;
    dIy = mask .* dIy;
    Grad = double([dIx(:) dIy(:)]);

    % compute steepest descent image
    SD = [Grad(:,1).*Xpoints Grad(:,2).*Xpoints Grad(:,1).*Ypoints Grad(:,2).*Ypoints Grad(:,1) Grad(:,2)];
    
    % compute Hessian
    H = SD' * SD;
    size(H);
    Esim = SD' * error;
    size(Esim);
    
    % compute deltaP
    deltaP = H\Esim;
    size(deltaP);
    param = param + deltaP';
    
%     % compute inserve
%     det = (1+param(1))*(1+param(4)) - param(2)*param(3);
%     invdeltaP = (1/det)*[-param(1)-param(1)*param(4)+param(2)*param(3);
%                                     -param(2);
%                                     -param(3);
%                                     -param(4)-param(1)*param(4)+param(2)*param(3);
%                                     -param(5)-param(1)*param(5)+param(3)*param(6);
%                                     -param(6)-param(1)*param(6)+param(2)*param(5);];
%                                 
%     % update warp function
%     param = [param(1)+invdeltaP(1)+param(1)*invdeltaP(1)+param(3)*invdeltaP(2);
%                    param(2)+invdeltaP(2)+param(2)*invdeltaP(1)+param(4)*invdeltaP(2);
%                    param(3)+invdeltaP(3)+param(1)*invdeltaP(3)+param(3)*invdeltaP(4);
%                    param(4)+invdeltaP(4)+param(2)*invdeltaP(3)+param(4)*invdeltaP(4);
%                    param(5)+invdeltaP(5)+param(1)*invdeltaP(5)+param(3)*invdeltaP(6);
%                    param(6)+invdeltaP(6)+param(2)*invdeltaP(5)+param(4)*invdeltaP(6)];
end
M = [1+param(1) param(3) param(5);
        param(2) 1+param(4) param(6);
        0             0                  1          ];
end
