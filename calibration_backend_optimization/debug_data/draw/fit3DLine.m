function [A, B] = fit3DLine (points3DLine)
avg = mean(points3DLine, 1);
subtracted = bsxfun(@minus, points3DLine, avg);
[~, ~, V] = svd(subtracted);
direction = V(:, 1);
A = avg';
B = direction;
endfunction
