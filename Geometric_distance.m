
 %% general parameter
 xmin = -2.0; xmax = +3.0;
 ymin = -3.0; ymax = +2.0;
 zmin = +0.0; zmax = +2.5;

 height = 480;
 width = 640;

 figure(1); hold on; grid on; axis equal;
 xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
 axis([xmin xmax ymin ymax zmin zmax]);
 view(60, 25);

 %% extrinsic camera parameter
 % camera centres
 C1 = [ 2.6607; 0.1006; 2.4089]; % Smart Sensor

 % rotation matrices
 R1 = [ -0.7770 0.6286 0.0339 0.3719 0.5018 -0.7809 -0.5079 -0.5942 -0.6237];

 % translation vectors
 T1 = -R1*C1;

 % setup homographies
 H1 = [R1 T1; 0 0 0 1];

 % plot WCS
 i = [1; 0; 0];
 j = [0; 1; 0];
 k = [0; 0; 1];
 o = [0; 0; 0];

plot3([o(1) i(1)], [o(2) i(2)], [o(3) i(3)], 'r-', 'LineWidth', 2);
plot3([o(1) j(1)], [o(2) j(2)], [o(3) j(3)], 'g-', 'LineWidth', 2);
plot3([o(1) k(1)], [o(2) k(2)], [o(3) k(3)], 'b-', 'LineWidth', 2);
text(o(1), o(2), o(3), ' WCS');

% plot camera coordinate systems
 i1 = H1^-1*[i; 1];
 j1 = H1^-1*[j; 1];
 k1 = H1^-1*[k; 1];
 o1 = H1^-1*[o; 1];

 plot3([o1(1) i1(1)], [o1(2) i1(2)], [o1(3) i1(3)], 'r-', 'LineWidth',2);
 plot3([o1(1) j1(1)], [o1(2) j1(2)], [o1(3) j1(3)], 'g-', 'LineWidth',2);
 plot3([o1(1) k1(1)], [o1(2) k1(2)], [o1(3) k1(3)], 'b-', 'LineWidth',2);
 text(o1(1), o1(2), o1(3), ' Cam 1');
% setup camera matices
 f = 500.0e-3; % 5mm
 px = 250.0e-5; % 5um
 py = 250.0e-5; % 5um
 cx = (width + 1)/2;
 cy = (height + 1)/2;
 ax = f/px; ay = f/py;
 K = [ax 0 cx; 0 ay cy; 0 0 1];
 P1 = K*R1*[eye(3) -C1]; P1 = P1./P1(end);

 % draw planes
 [X, Y, Z] = meshgrid(xmin:0.1:xmax,
ymin:0.1:ymax, zmin:0.1:zmax);
 Xwrld = [X(:)'; Y(:)'; Z(:)'; ones(1,numel(X))];

 Computation of the Camera Matrix P

 % world plane
 plane = [0; 0; 1; 0];
 data = plane'*Xwrld;
 data = reshape(data, size(X));
 p = patch(isosurface(X, Y, Z, data, 0)); hold on;
 set(p, 'FaceColor', 'black', 'EdgeColor', 'none')
 alpha(0.25);

 % image planes
 
 [x, y] = meshgrid(1:1:width, 1:1:height);
 Ximg = [x(:)'; y(:)'; ones(1, numel(x))];
 Xnorm = K^-1*Ximg;
 Xcam = [Xnorm.*f; ones(1, size(Xnorm, 2))];
 Xwrld1 = H1^-1*Xcam;
 h1 = surf(reshape(Xwrld1(1, :), size(x)), ...
 reshape(Xwrld1(2, :), size(x)), reshape(Xwrld1(3,:), size(x)), 'FaceColor', 'black', 'EdgeColor', 'none'); alpha(h1, 0.25);

 % generate world points
 Xwrld = [ -1.0 +1.0 +1.0 -1.0 -1.0 +1.0 +1.0 -1.0 0.0 +3.0 +3.0 -2.5 -2.5 -0.5 -0.5 -2.5 -2.5 -0.5 -0.5 -1.5 -0.5 -2.5 0.0 0.0 0.0 0.0 +1.0+1.0 +1.0 +1.0 +2.0 0.0 0.0
 +1.0 +1.0 +1.0 +1.0 +1.0
+1.0 +1.0 +1.0 +1.0 +1.0 +1.0];

 plot3(Xwrld(1, :), Xwrld(2, :), Xwrld(3,:),'m*', 'MarkerSize', 5);

 Conn = [1 1 2 2 3 3 4 4 5 6 7 8 5 6 7 8
 2 5 3 6 4 7 1 8 6 7 8 5 9 9 9 9];