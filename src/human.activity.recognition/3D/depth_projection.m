function [F,S,T] = depth_projection(X)
[rows, cols, D] = size(X);
X2D = reshape(X, rows*cols, D);
max_depth = max(X2D(:));

F = zeros(rows, cols);
S = zeros(rows, max_depth);
T = zeros(max_depth, cols);

for k = 1:D   
    front = X(:,:,k);
    side = zeros(rows, max_depth);
    top = zeros(max_depth, cols);
    for i = 1:rows
        for j = 1:cols
            if front(i,j) ~= 0
                side(i,front(i,j)) = j;   % side view projection (y-z projection)
                top(front(i,j),j)  = i;   % top view projection  (x-z projection)
            end
        end
    end
    
    if k > 1
        F = F + (k^2)*abs(front - front_pre);
        S = S + (k^2)*abs(side - side_pre);
        T = T + (k^2)*abs(top - top_pre);
    end   
    front_pre = front;
    side_pre  = side;
    top_pre   = top;
end

F1 = bounding_box(F);
S1 = bounding_box(S);
T1 = bounding_box(T);

figure 
imshow(F);
figure 
imshow(S);
figure 
imshow(T);

F = norm(F1);
S = norm(S1);
T = norm(T1);

figure 
imshow(F);
figure 
imshow(S);
figure 
imshow(T);
end

function [im2] = norm(im1)
sz = size(im1);
im2 = zeros(sz(1), sz(2));    
ma = max(max(im1));
for m=1:sz(1)
    for n=1:sz(2)
        im2(m,n) = im1(m,n)/ma;
    end
end
% figure
% imshow(im2);
end 