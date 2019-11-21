
% Camera Estimation from a Calibration Object

function [X_norm, T] = Norm3Dpoint(X)
% Number of Points
no_pts = size(X, 2);
% Normalize points for estimation (isotropic scaling)
% move centroid to zero
Centr = mean(X(1:3, :), 2);
X_cent = bsxfun(@minus, X(1:3, :), Centr);
% change mean distance to sqrt(2) from origin, respectively sqrt(3)
X_dist = bsxfun(@power, X_cent(1, :).^2 + X_cent(2, :).^2 ...
+ X_cent(3, :).^2, 1/2);
s = sqrt(3)/mean(X_dist);
X_norm = [s*X_cent(1:3, :); ones([1 no_pts])];
% Composition of the normalization matrix.
T = diag([s s s 1]);
T(1:3, 4) = -s*Centr;
return % end of Norm2Dpoint

function P = EstimPNormML(Ximg, Xwrld, Pest)
% Initialization and Normalization
[Ximg_norm, T1] = Norm2Dpoint(Ximg); % Perfect points
[Xwrld_norm, T2] = Norm3Dpoint(Xwrld); % Measurement Space: N = 3n
Pest_norm = T1*Pest*T2^-1;
p0 = reshape(Pest_norm', 1, []); % initial estimate of P
% Optimization
[p_opt, ~, ~, ~, ~] = LMopt(p0, Ximg_norm, Xwrld_norm);
Popt_tilde = reshape(p_opt, 4, 3)';
Popt_tilde = Popt_tilde./Popt_tilde(3, 3);
% Denormalization
P = T1^-1*Popt_tilde*T2;

% Denormalization
P = T1^-1*Popt_tilde*T2;
function [x, resnorm, residual, exitflag, output] = LMopt(x0, Ximg, Xwrld)
options = optimset('MaxFunEvals', 5000, ...
'Algorithm', {'levenberg-marquardt', .005}, 'TolX', 1e-7);
[x, resnorm, residual, exitflag, output] = ...
lsqnonlin(@costfun, x0, [], [], options);
function [e] = costfun(x)
P = reshape(x, 4, 3)';
% Transform estimated points
Ximg_est = P*Xwrld;
Ximg_est = bsxfun(@rdivide, Ximg_est, Ximg_est(3, :));
e = [ ...
bsxfun(@minus, Ximg(1, :), Ximg_est(1, :)), ...
bsxfun(@minus, Ximg(2, :), Ximg_est(2, :)), ...
];
end
end
end
