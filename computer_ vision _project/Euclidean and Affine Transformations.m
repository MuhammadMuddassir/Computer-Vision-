 %  Euclidean and Affine Transformations - Translation , Rotation and Scaling 
 %  2-by-11 matrix to draw a simple house
 X = [ -6 -6 -7 0 7 6 6 -3 -3 0 0 -7 2 1 8 1 2 -7 -7 -2 -2 -7 ];
 no_pts = size(X, 2);
 h1 = figure(1); plot(X(1, :), X(2, :), 'ob'),hold on;
 title('A Simple House', 'FontWeight', 'bold','FontSize', 12);

 for i=1:no_pts
 x1 = X(:, i);
 x2 = X(:, mod(i, no_pts) + 1);
 plot([x1(1) x2(1)], [x1(2) x2(2)], '-b')
 end

 % points homogeneous
 X = [X; ones(1, size(X, 2))];

 % Projectivity 1
 H1 = [ 1 0 0;0 1 0;0.04 0.00 1];
 % Projectivity 2
 H2 = [ 1 0 0; 0 1 0; 0.04 0.04 1];

 disp(H1); disp(H2);
% Apply transformation
 X1 = H1*X;
 X2 = H2*X;
 disp(X1); disp(X2);

 %  coordinates inhomogeneous
 X1 = bsxfun(@rdivide, X1(1:2, :), X1(3, :));
 X2 = bsxfun(@rdivide, X2(1:2, :), X2(3, :));
 X1 = X1(1:2, :);
 X2 = X2(1:2, :);

 % Plot transformed houses
 h2 = figure(2);
 plot(X1(1, :), X1(2, :), 'or'); hold on;
 plot(X2(1, :), X2(2, :), 'og'); hold on;
 title('Some Simple Transformations',
'FontWeight', 'bold',
'FontSize', 12);
 legend('Projectivity 1', 'Projectivity 2');
 
 for i=1:no_pts

 x1 = X1(:, i);
 x2 = X1(:, mod(i, no_pts) + 1);
 plot([x1(1) x2(1)], [x1(2) x2(2)], '-r');

 x1 = X2(:, i);
 x2 = X2(:, mod(i, no_pts) + 1);
 plot([x1(1) x2(1)], [x1(2) x2(2)], '-g');
 end
 axis equal

 print(h1, '-r600', '-dtiff', '03.tif')
 print(h2, '-r600', '-dtiff', '04.tif')
 highlight('m02.m', 'rtf', 'm02.rtf')
 
 
 
%% Perspective Projection - The Projection Matrix P
%% Camera Matrix
 % Intrinsic Camera Parameter
 f = 2; % Focal Length
 K = diag([f f 1]); % Intrinsic camera Parameter
 K(1, 3) = 0;
 K(2, 3) = 0;

 % Extrinsic Camera Parameter
 C = [0; 0; 0]; % Camera Centre
 R = [1 0 0; 0 0 1; 0 1 0]; % We change x and y coordinates

%% Camera Matrix
% Intrinsic Camera Parameter
 f = 2; % Focal Length
 K = diag([f f 1]); % Intrinsic camera Parameter
 K(1, 3) = 3;
 K(2, 3) = 2;

%% Perspective Projection - The Projection Matrix P - Basler acA640-100gc
 
 %% Example Projection Matrix (Computed)

 fx = 4.5e-3; % Focal Length 4.5mm
 fy = 4.5e-3; % Focal Length 4.5mm
 mx = 1/6.5e-6; % no of pixels per mm
 my = 1/6.5e-6; % no of pixels per mm
 x0 = 640/2;
 y0 = 480/2;
 ax = fx*mx;
 ay = fy*my;
 s = 0; % skew factor

 K = [ ax s x0; 0 0 1 ];

 C = [0; 0; 0];
 R = eye(3);
 t = -R*C;
 P = K*[R t];
 disp(P);
 % Extrinsic Camera Parameter
 C = [3; 2; 1]; % Camera Centre
 R = [1 0 0; 0 0 1; 0 1 0]; % We change x and y coordinates