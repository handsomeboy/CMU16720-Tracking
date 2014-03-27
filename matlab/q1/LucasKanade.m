function [ u, v ] = LucasKanade( It, It1, rect )
% Lucas-Kanade algorithm to compute optimal local motion
% Inputs:
% It - image frame at time t
% It1 - image frame at time t+1
% rect - vector representing a rectangle on the image frame It
% Outputs:
% u,v - optimal optical flow

% initializtion
u = 0;
v = 0;
epsilon = 0.01;
deltaP = [1 1];

% get template image
[XT, YT] = meshgrid(rect(1):rect(3), rect(2):rect(4));
template = interp2(It, XT, YT);

% evaluate the gradient of template image
[dTx, dTy] = gradient(template);

% compute Jacobian and steepest descent
% here Jacobian is an identity matrix
warpGrad = double([dTx(:), dTy(:)]);

% compute Hessian matrix
H = warpGrad' * warpGrad;

count = 0;
%while(norm(deltaP) > epsilon)
while(count < 50)
    count = count + 1;
    
    % warp image with W(x;p) to compute I(W(x;p))
    x = (rect(1)+u) : (rect(3)+u);
    y = (rect(2)+v) : (rect(4)+v);
    [X, Y] = meshgrid(x, y);
    warpimg = interp2(It1, X, Y);
    
    % compute error
    error = warpimg - template;
    error = error(:);
    
    % compute deltaP
    deltaP = H\(warpGrad' * error);
    if isnan(deltaP)
        count;
    end
    
    % update warp
    u = u - deltaP(1);
    v = v - deltaP(2);
    
    if (norm(deltaP) > epsilon)
        continue
    end
    
end
end
