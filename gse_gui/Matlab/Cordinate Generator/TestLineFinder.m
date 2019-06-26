clear;

I = imread('InputPhotos/GUILineBlackPNG.png');
I = rgb2gray(I);
I = imcomplement(I);
[H,theta,rho] = hough(I);

P = houghpeaks(H,150,'threshold',ceil(.01*max(H(:))), 'NHoodSize',[71 71]);

x = theta(P(:,2));
y = rho(P(:,1));

lines = houghlines(I,theta,rho,P,'FillGap',50,'MinLength',20);

figure, imshow(I), hold on
max_len = 0;

VertlineData = {};
HorLineData = {};

for k = 1:length(lines)
   xy = [lines(k).point1; lines(k).point2];
   slope = (xy(2,2) - xy(1,2)) / (xy(1,1) - xy(2,1));
   
   
   
   if isinf(slope) == 1 || abs(slope) > 10
        plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');
       %Plot beginnings and ends of lines
       plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
       plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');
   
       VertlineData{end + 1} = {[xy(1,1), xy(2,1)], [xy(1,2), xy(2,2)], slope, norm(lines(k).point1 - lines(k).point2), 1};
   elseif isinf(slope) == 0 && abs(slope) < .2
       HorLineData{end + 1} = {[xy(1,1), xy(2,1)], [xy(1,2), xy(2,2)], slope, norm(lines(k).point1 - lines(k).point2), 1};
   end
   %Determine the endpoints of the longest line segment
   len = norm(lines(k).point1 - lines(k).point2);
   if ( len > max_len)
      max_len = len;
      xy_long = xy;
   end
end
% highlight the longest line segment
plot(xy_long(:,1),xy_long(:,2),'LineWidth',2,'Color','red');
hold off;

%figure, imshow(I), hold on
similarX = [];
similarXI = 0;
maxVL = [];
a = 1;
k = 2;

for i = 1:length(VertlineData)
    j = VertlineData{i}{1}(1):VertlineData{i}{1}(2)
    if VertlineData{i}{1}(1) > VertlineData{i}{1}(2)
       a = VertlineData{i}{1}(1);
       VertlineData{i}{1}(1) =  VertlineData{i}{1}(2);
       VertlineData{i}{1}(2) = a;
    end
    b = VertlineData{i}{1}(1):VertlineData{i}{1}(2)
end


for i = 1:length(VertlineData)
    while k <= length(VertlineData)
        if k ~= i  
            VertlineData{k}{1}(1):VertlineData{k}{1}(2)
            if sum(intersect(VertlineData{i}{1}(1):VertlineData{i}{1}(2), VertlineData{k}{1}(1):VertlineData{k}{1}(2))) > 0
                if VertlineData{k}{4} >= VertlineData{i}{4}
                    VertlineData{i}{5} = 0;
                    %VertlineData{k}{5} = 1;
                elseif VertlineData{i}{4} > VertlineData{k}{4}
                    VertlineData{k}{5} = 0;
                end
            end
        end
        k = k + 1;
    end
    k = a + 1;
    a = k;
end


figure, imshow(I), hold on
for i = 1:length(VertlineData)
    if VertlineData{i}{3} < 0 && isinf(VertlineData{i}{3}) == 0
       'te'
       %VertlineData{i}{5} = 0;
    end
    
    if VertlineData{i}{5} == 1
        %'yo'
        text(VertlineData{i}{1}(1), VertlineData{i}{1}(2), '1000000000')
        plot([VertlineData{i}{1}(1) VertlineData{i}{1}(2)], [VertlineData{i}{2}(1) VertlineData{i}{2}(2)] ,'LineWidth',2,'Color','green');
    end
end

