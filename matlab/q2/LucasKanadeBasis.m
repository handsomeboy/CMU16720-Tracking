function [ u, v ] = LucasKanadeBasis( It, It1, rect, basis )
% Lucas-Kanade Tracking with Appearance Basis
% Inputs:
% It - image frame at time t
% It1 - image frame at time t+1
% rect - vector representing a rectangle on the image frame It
% basis - a cell array of bases
% Outputs:
% u,v - optimal optical flow

% initializtion
u = 0;
v = 0;
epsilon = 0.01;
deltaP = [1 1];
lambda = zeros(1,length(basis));
%rect_orig = rect;

% get template image
[XT, YT] = meshgrid(rect(1):rect(3), rect(2):rect(4));
template = interp2(It, XT, YT);

% evaluate the gradient of template image
[dTx, dTy] = gradient(template);
GradT = double([dTx(:), dTy(:)]);

% evaluate the gradient of the basis
A = zeros(size(basis{1},1) * size(basis{1},2), length(basis));
GradA = zeros(size(A,1), size(A,2)*2);
for i = 1:length(basis)
    tempBasis = basis{i};
    A(:, i) = tempBasis(:);
    [dAx, dAy] = gradient(tempBasis);
    GradA(:, 2*i-1) = double(dAx(:));
    GradA(:, 2*i) = double(dAy(:));
end

% compute Jacobian at (x;0)
Jacob = [1 0; 0 1];

count = 0;
%while(norm(deltaP) > epsilon)
while(count < 50 || norm(deltaP) > epsilon)
    count = count + 1;
    if count > 50
        count
    end
    % warp image with W(x;p) to compute I(W(x;p))
    x = (rect(1)+u) : (rect(3)+u);
    y = (rect(2)+v) : (rect(4)+v);
    [X, Y] = meshgrid(x, y);
    warpimg = interp2(It1, X, Y);

    % compute error
    sumA = 0;
    for i = 1:length(basis)
        sumA = sumA + lambda(i) * A(:,i);
    end
    sumA = reshape(sumA, size(template));
    error = template + sumA - warpimg;
    error = error(:);

    % compute steepest descent image
    sumGrad = 0;
    for i = 1:length(basis)
        tempA = [GradA(:,2*i-1), GradA(:,2*i)];
        sumGrad = sumGrad + lambda(i)*tempA;
    end
    %size(sumGrad)
    sumGrad = sumGrad + GradT;
    SD = [sumGrad*Jacob(:,1) sumGrad*Jacob(:,2) A];
    
    % compute Hessian
    H = SD'*SD;
    Esim = SD'*error;
  
    % compute deltaP
    deltaq = -H\Esim;
    deltaP = deltaq(1:2);
    
    % update warp
    u = u - deltaP(1);
    v = v - deltaP(2);
    lambda = lambda + deltaq(3:end)';
    
    if (norm(deltaP) > epsilon)
        continue
    end
end

%u = rect(1)-rect_orig(1);
%v = rect(2)-rect_orig(2);

end
