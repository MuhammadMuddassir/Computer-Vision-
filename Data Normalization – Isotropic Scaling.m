% Data Normalization – Isotropic Scaling
%general parameter
 a = 20;
 b = 10;
 Ximg = (b-a).*rand(2, 10) + a;

 figure(1); hold on; grid on; axis equal;
 xlabel('x'); ylabel('y');
 plot(Ximg(1, :), Ximg(2, :), 'ro');

 %% Number of Points
 no_pts = size(Ximg, 2);

 %% Normalize points for estimation (isotropic scaling)
 % move centroid to zero
 centr = mean(Ximg(1:2, :), 2);
 X_cent = bsxfun(@minus, Ximg(1:2, :), centr);

 % change mean distance to sqrt(2) from origin, respectively sqrt(3)
 X_dist = bsxfun(@power, X_cent(1, :).^2 + X_cent(2, :).^2, 1/2);
 s = sqrt(2)/mean(X_dist);
 X_norm = [s*X_cent(1:2, :); ones([1 no_pts])];

% Computation of the Camera Matrix P
% Data Normalization – Isotropic Scaling
% Composition of the normalization matrix.
 T = diag([s s 1]);
 T(1:2, 3) = -s*centr;

 plot(X_norm(1, :), X_norm(2, :), 'go');

 for i=1:10
 plot([Ximg(1, i) X_norm(1, i)], [Ximg(2, i)
X_norm(2, i)], 'k--');
 end