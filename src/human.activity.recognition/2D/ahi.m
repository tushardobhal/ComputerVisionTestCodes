function [] = ahi(class)
global im2;
bg(sprintf('Weizmannn Dataset/%s/daria_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/daria_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/denis_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/denis_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/eli_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/eli_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/ido_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/ido_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/ira_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/ira_%s.jpg', class, class));

if(strcmp(class, 'run') || strcmp(class, 'walk') || strcmp(class, 'skip'))
      bg(sprintf('Weizmannn Dataset/%s/lena_%s1.avi', class, class));
      im3 = bounding_box(im2);
      imwrite(im3, sprintf('Weizmannn Dataset/%s/lena_%s1.jpg', class, class));
      bg(sprintf('Weizmannn Dataset/%s/lena_%s2.avi', class, class));
      im3 = bounding_box(im2);
      imwrite(im3, sprintf('Weizmannn Dataset/%s/lena_%s2.jpg', class, class));
else
      bg(sprintf('Weizmannn Dataset/%s/lena_%s.avi', class, class));
      im3 = bounding_box(im2);
      imwrite(im3, sprintf('Weizmannn Dataset/%s/lena_%s.jpg', class, class));
end

bg(sprintf('Weizmannn Dataset/%s/lyova_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/lyova_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/moshe_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/moshe_%s.jpg', class, class));

bg(sprintf('Weizmannn Dataset/%s/shahar_%s.avi', class, class));
im3 = bounding_box(im2);
imwrite(im3, sprintf('Weizmannn Dataset/%s/shahar_%s.jpg', class, class));
end

function [] = init()
     global im1;
     global im2;
     global sz;
     sz = [120,160];
     im1 = zeros(sz(1),sz(2));
     im2 = zeros(sz(1),sz(2));
end

function [] =  compAhi(im, k)

%x = 50;
%y = 71;
%n = y-x+1;
%I1 = zeros(sz(1),sz(2),n);
%for i=x:x+n-1
%    im = imread(sprintf('1/%d.jpg',i));
    %im = rgb2gray(im);
    %im = imresize(im,sz); 
%    if(rem(i,2) == 0)
%     I1 = [I1, im/255];
%    end 
%end

global im1;
global sz;

for i=1:sz(1)
    for j=1:sz(2)
            im1(i,j) = im1(i,j) + (k^2)*(im(i,j));
    end
end
end

function [] = norm()
global im1;
global im2;
global sz;

m = max(max(im1));
for i=1:sz(1)
    for j=1:sz(2)
        im2(i,j) = im1(i,j)/m;
    end
end
%figure
%imshow(im2);
end 

function [] = bg(name)
videoSource = vision.VideoFileReader(name,'ImageColorSpace','Intensity','VideoOutputDataType','uint8');
detector = vision.ForegroundDetector('NumTrainingFrames', 20, 'InitialVariance', 30*30); % initial standard deviation of 30

%videoPlayer = vision.VideoPlayer();
flag = 0;
i = 0;
se = strel('disk', 3);
init();
while ~isDone(videoSource)
     frame  = step(videoSource);
     fgMask = step(detector, frame);
     fgMask = imerode(fgMask, se);
     fgMask = imdilate(fgMask, se);
     numPix = numPixel(fgMask);
     %if(numPix > 10)
         flag = 1;
         i = i+1; 
         compAhi(fgMask, i);
     %else
         %if(flag == 1)
             %norm();
             %i = 0;
             flag = 0;
             %init();
         %end
     %end
     %step(videoPlayer, fgMask);
end
norm();
%release(videoPlayer);
release(videoSource);
end

function [d] = numPixel(fg)
  d = 0;
  for i = 1:size(fg,1)
      for j = 1:size(fg,2)
          if(fg(i,j) > 0)
              d = d + 1;
          end
      end
  end
end

function y = bounding_box(img)

[row col] = size(img);

for i = 1:row
    if sum(img(i,:)) > 0
        top = i;
        break
    end
end

for i = row:(-1):1
    if sum(img(i,:)) > 0
        bottom = i;
        break
    end
end

for i = 1:col
    if sum(img(:,i)) > 0
        left = i;
        break
    end
end

for i = col:(-1):1
    if sum(img(:,i)) > 0
        right = i;
        break
    end
end

y = img(top:bottom, left:right);
end