load('../data/bookSequence.mat');
rect = [247 102 285 161];
rectb = [247 102 285 161];
width = abs(rect(1) - rect(3));
height = abs(rect(2) - rect(4));
[h,w,channels,frames] = size(sequence);
box = zeros(frames, 4);
box(1, :) = rectb;

for i = 1:frames-1
    i;
    
    It = mat2gray(rgb2gray(sequence(:,:,:,i)));
    It1 = mat2gray(rgb2gray(sequence(:,:,:,i+1)));
    [ub, vb] = LucasKanadeBasis(It, It1, rectb, basis);
    [u, v] = LucasKanade(It, It1, rect);
    
    rectb = [rectb(1)+ub rectb(2)+vb rectb(3)+ub rectb(4)+vb];
    rect = [rect(1)+u rect(2)+v rect(3)+u rect(4)+v];
    
    rectb = round(rectb);
    rect = round(rect);
    
    img = im2double(sequence(:,:,:,i));
    imshow(img);
    
    hold on;
    rectangle('Position',[rectb(1), rectb(2), width, height], 'LineWidth',2, 'EdgeColor', 'g');
    rectangle('Position',[rect(1), rect(2), width, height], 'LineWidth',2, 'EdgeColor', 'r');
    pause(0.01);
    
    if i==29 || i== 149 || i==247
        filename = sprintf('basicframe%d.jpg',i+1);
        set(gcf, 'name', filename);
        saveas(gcf, filename, 'jpg');
    end
    
    box(i+1, :) = rectb;
    
end

save('bookPosition.mat', 'box');