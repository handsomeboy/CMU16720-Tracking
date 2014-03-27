load('../data/carSequence.mat');
rect = [328 213 419 265];
width = abs(rect(1) - rect(3));
height = abs(rect(2) - rect(4));
[h,w,channels,frames] = size(sequence);
box = zeros(frames, 4);
box(1, :) = rect;

for i = 1:frames-1
    img = im2double(sequence(:,:,:,i));
    imshow(img);
    
    hold on;
    rectangle('Position',[rect(1), rect(2), width, height], 'LineWidth',2, 'EdgeColor', 'g');
    hold off;
    pause(0.01);
    
    It = rgb2gray(im2double(sequence(:,:,:,i)));
    It1 = rgb2gray(im2double(sequence(:,:,:,i+1)));
    [u, v] = LucasKanade(It, It1, rect);
    
    rect = [rect(1)+u rect(2)+v rect(3)+u rect(4)+v];
    rect = round(rect);
    
     if i==19 || i== 49 || i==99
        filename = sprintf('carframe%d.jpg',i+1);
        set(gcf, 'name', filename);
        saveas(gcf, filename, 'jpg');
    end
    
    box(i+1, :) = rect;
    
end

save('carPosition.mat', 'box');