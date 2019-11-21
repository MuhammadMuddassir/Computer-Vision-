%% Basic Equations/DLT


 %% general parameter
 xmin = -3.0; xmax = +3.0;
 ymin = -3.0; ymax = +3.0;
 zmin = +0.0; zmax = +2.5;

 height = 480;
 width = 640;

 figure(1); hold on; grid on; axis equal;
 xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
 axis([xmin xmax ymin ymax zmin zmax]);
 view(60, 25);

 %% extrinsic camera parameter
 % camera centres
 C1 = [ 2.6607; 0.1006; 2.4089]; % Smart Sensor 605
 C2 = [-1.6157; 2.7995; 2.4119]; % Smart Sensor 601
 C3 = [-1.8482; -1.7443; 2.3955]; % Smart Sensor 602

 % rotation matrices
 R1 = [ -0.7770 0.6286 0.0339 0.3719 0.5018 -0.7809  -0.5079 -0.5942 -0.6237];
 R2 = [ -0.5233 -0.8521 0.0080  -0.5388 0.3236 -0.7778 0.6602 -0.4113 -0.6285];
 R3 = [ -0.4861 -0.8670 0.1097 -0.6689 0.2884 -0.6852 0.5624 -0.4065 -0.7201];

 % translation vectors
 T1 = -R1*C1;
 T2 = -R2*C2;
 T3 = -R3*C3;

 %% setup homographies
 H1 = [R1 T1; 0 0 0 1];
 H2 = [R2 T2; 0 0 0 1]; 
 H3 = [R3 T3; 0 0 0 1];

 %% plot WCS
 i = [1; 0; 0];
 j = [0; 1; 0];
 k = [0; 0; 1];
 o = [0; 0; 0];

 plot3([o(1) i(1)], [o(2) i(2)], [o(3) i(3)], 'r-','LineWidth', 2);
 plot3([o(1) j(1)], [o(2) j(2)], [o(3) j(3)], 'g-','LineWidth', 2);
 plot3([o(1) k(1)], [o(2) k(2)], [o(3) k(3)], 'b-','LineWidth', 2);
 text(o(1), o(2), o(3), ' WCS');

 %% plot camera coordinate systems
 i1 = H1^-1*[i; 1]; 
 j1 = H1^-1*[j; 1];
 k1 = H1^-1*[k; 1];
 o1 = H1^-1*[o; 1];
 plot3([o1(1) i1(1)], [o1(2) i1(2)], [o1(3) i1(3)], 'r-','LineWidth', 2);
 plot3([o1(1) j1(1)], [o1(2) j1(2)], [o1(3) j1(3)], 'g-','LineWidth', 2);
 plot3([o1(1) k1(1)], [o1(2) k1(2)], [o1(3) k1(3)], 'b-','LineWidth', 2);
 text(o1(1), o1(2), o1(3), ' Cam 1');

 Computation of the Camera Matrix P
 i2 = H2^-1*[i; 1];
 j2 = H2^-1*[j; 1];
 k2 = H2^-1*[k; 1];
 o2 = H2^-1*[o; 1];

 plot3([o2(1) i2(1)], [o2(2) i2(2)], [o2(3) i2(3)], 'r-','LineWidth', 2);
 plot3([o2(1) j2(1)], [o2(2) j2(2)], [o2(3) j2(3)], 'g-','LineWidth', 2);
 plot3([o2(1) k2(1)], [o2(2) k2(2)], [o2(3) k2(3)], 'b-','LineWidth', 2);
 text(o2(1), o2(2), o2(3), ' Cam 2');
 i3 = H3^-1*[i; 1];
 j3 = H3^-1*[j; 1];
 k3 = H3^-1*[k; 1];
 o3 = H3^-1*[o; 1];

 plot3([o3(1) i3(1)], [o3(2) i3(2)], [o3(3) i3(3)], 'r-','LineWidth', 2);
 plot3([o3(1) j3(1)], [o3(2) j3(2)], [o3(3) j3(3)], 'g-','LineWidth', 2);
 plot3([o3(1) k3(1)], [o3(2) k3(2)], [o3(3) k3(3)], 'b-','LineWidth', 2);
 text(o3(1), o3(2), o3(3), ' Cam 3');

 %% setup camera matices
 f = 500.0e-3; % 5mm
 px = 250.0e-5; % 5um
 py = 250.0e-5; % 5um
 cx = (width + 1)/2;
 cy = (height + 1)/2;

 ax = f/px; ay = f/py;
 K = [ax 0 cx; 0 ay cy; 0 0 1];

 P1 = K*R1*[eye(3) -C1]; % P1 = P1./P1(end);
 P2 = K*R2*[eye(3) -C2]; % P2 = P2./P2(end);
 P3 = K*R3*[eye(3) -C3]; % P3 = P3./P3(end);
 %% draw planes
 [X, Y, Z] = meshgrid(xmin:0.1:xmax, ymin:0.1:ymax,zmin:0.1:zmax);
 Xwrld = [X(:)'; Y(:)'; Z(:)'; ones(1, numel(X))];

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
 Xwrld2 = H2^-1*Xcam;
 Xwrld3 = H3^-1*Xcam;
 
 h1 = surf(reshape(Xwrld1(1, :), size(x)), ...
 reshape(Xwrld1(2, :), size(x)), reshape(Xwrld1(3, :),size(x)), 'FaceColor', 'black', 'EdgeColor', 'none');
 h2 = surf(reshape(Xwrld2(1, :), size(x)), reshape(Xwrld2(2, :), size(x)), reshape(Xwrld2(3, :),
size(x)),'FaceColor', 'black', 'EdgeColor', 'none');
h3 = surf(reshape(Xwrld3(1, :), size(x)), reshape(Xwrld3(2, :), size(x)), reshape(Xwrld3(3, :),
size(x)), 'FaceColor', 'black', 'EdgeColor', 'none');
 alpha(h1, 0.25); alpha(h2, 0.25); alpha(h3, 0.25);
 %% generate world points
 Xwrld = [ -1.0 +1.0 +1.0 -1.0 -1.0 +1.0 +1.0 -1.0 0.0 +3.0 +3.0 -2.5 -2.5 -0.5 -0.5 -2.5 -2.5 -0.5 -0.5 -1.5 -0.5 -2.5 0.0 0.0 0.0 0.0 +1.0 +1.0 +1.0 +1.0 +2.0 0.0 0.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0 +1.0];
0149 plot3(Xwrld(1, :), Xwrld(2, :), Xwrld(3, :), 'm*',
'MarkerSize', 5);

 Conn = [1 1 2 2 3 3 4 4 5 6 7 8 5 6 7 8
 2 5 3 6 4 7 1 8 6 7 8 5 9 9 9 9];

 for i=1:size(Conn, 2);
 Xwrld1 = Xwrld(:, Conn(1, i));
 Xwrld2 = Xwrld(:, Conn(2, i));
 plot3([Xwrld1(1) Xwrld2(1)], [Xwrld1(2) Xwrld2(2)], ...
 [Xwrld1(3) Xwrld2(3)], '-m')
 end

 %% generate images
 Ximg1 = P1*Xwrld;
 Ximg2 = P2*Xwrld;
 Ximg3 = P3*Xwrld;

 Ximg1 = bsxfun(@rdivide, Ximg1, Ximg1(3, :));
 Ximg2 = bsxfun(@rdivide, Ximg2, Ximg2(3, :));
 Ximg3 = bsxfun(@rdivide, Ximg3, Ximg3(3, :));

 Xnorm1 = K^-1*Ximg1;
 Xcam1 = [Xnorm1.*f; ones(1, size(Xnorm1, 2))];
 Xwrld1 = H1^-1*Xcam1;
 plot3(Xwrld1(1, :), Xwrld1(2, :), Xwrld1(3, :), 'm*',
'MarkerSize', 5);

 for i=1:size(Conn, 2);
 Xwrld11 = Xwrld1(:, Conn(1, i));
 Xwrld12 = Xwrld1(:, Conn(2, i));
 plot3([Xwrld11(1) Xwrld12(1)], [Xwrld11(2) Xwrld12(2)], [Xwrld11(3) Xwrld12(3)], '-m')
 end
 
 
 %% Direct Linear Transformation (DLT) Algorithm

Plot Transformed Houses
 plot(X2(1, subset), X2(2, subset), 'og'); hold on;
 title('Four Point Correspondences', 'FontWeight',
'bold', ...
 'FontSize', 12);

 for i=1:no_pts
 x1 = X2(:, i);
 x2 = X2(:, mod(i, no_pts) + 1);
 plot([x1(1) x2(1)], [x1(2) x2(2)], '-g',
'LineWidth', 2);
 end
 for i=subset
 plot([X(1, i) X2(1, i)], [X(2, i) X2(2, i)],
'--r');
 end
 axis equal
 text(6, 5, ‘…', ...

%% Compute the Homography
 X2 = [X2; ones(1, size(X2, 2))];
 x_h_ = X2(:, subset);
 x_h = X(:, subset);
 no_pts = 4;

 A = [zeros([no_pts 3]) -x_h' bsxfun(@times, x_h_(2, :), x_h)'
 x_h' zeros([no_pts 3]) bsxfun(@times, -x_h_(1, :), x_h)'
 ];

 h = null(A);
 H = reshape(h, 3, 3)';
 H = H./H(end);
 'FontSize', 25, 'FontWeight','bold');
 print(h1, '-r600', '-dtiff', '03.tif')