clear;
clc;
close all;

%% Symbolic variables
% angle-axis
rx = sym('rx', 'real');
ry = sym('ry', 'real');
rz = sym('rz', 'real');
r = [rx; ry; rz;];
% translation
tx = sym('tx', 'real');
ty = sym('ty', 'real');
tz = sym('tz', 'real');
t = [tx; ty; tz;];
x = [r; t];
% transformed point and reference point
px = sym('px', 'real');
py = sym('py', 'real');
pz = sym('pz', 'real');
p = [px; py; pz];
qx = sym('qx', 'real');
qy = sym('qy', 'real');
qz = sym('qz', 'real');
q = [qx; qy; qz];
z = [p; q];
% point normal
nx = sym('nx', 'real');
ny = sym('ny', 'real');
nz = sym('nz', 'real');
n = [nx; ny; nz];

%% Linearised Rotation
% Rotation matrix I + [r]_x
R_lin = [  1, -rz,  ry;
      rz,   1, -rx;
     -ry,  rx,   1];

% Transform matrix
T_lin = [R_lin, t; 0 0 0 1];

%% Point-to-Point Linearised
f_lin = simplify(R_lin * p + t - q);
df_dx_lin = simplify(jacobian(f_lin, x));
dF_dx_lin = simplify(2.0 * df_dx_lin' * f_lin);
d2F_dx2_lin = simplify(jacobian(dF_dx_lin, x));
d2F_dzdx_lin = simplify(jacobian(dF_dx_lin, z));

% Save to file
fid = fopen("point_to_point_linear.txt", "w");
for i = 1:length(d2F_dx2_lin(1,:))
    for j = 1:length(d2F_dx2_lin(:,1))
        fprintf(fid, "d2F_dx2(%d, %d) = %s;\n", i-1, j-1, char(d2F_dx2_lin(i, j)));
    end
end
for i = 1:length(d2F_dzdx_lin(1,:))
    for j = 1:length(d2F_dzdx_lin(:,1))
        fprintf(fid, "d2F_dzdx(%d, %d) = %s;\n", i-1, j-1, char(d2F_dzdx_lin(i, j)));
    end
end

%% Point-to-Plane Linearised
h_lin = simplify(dot((R_lin * p + t - q), n));
dh_dx_lin = simplify(jacobian(h_lin, x));
dH_dx_lin = simplify(2.0 * dh_dx_lin' * h_lin);
d2H_dx2_lin = simplify(jacobian(dH_dx_lin, x));
d2H_dzdx_lin = simplify(jacobian(dH_dx_lin, z));

% Save to file
fid = fopen("point_to_plane_linear.txt", "w");
for i = 1:length(d2H_dx2_lin(1,:))
    for j = 1:length(d2H_dx2_lin(:,1))
        fprintf(fid, "d2H_dx2(%d, %d) = %s;\n", i-1, j-1, char(d2H_dx2_lin(i, j)));
    end
end
for i = 1:length(d2H_dzdx_lin(1,:))
    for j = 1:length(d2H_dzdx_lin(:,1))
        fprintf(fid, "d2H_dzdx(%d, %d) = %s;\n", i-1, j-1, char(d2H_dzdx_lin(i, j)));
    end
end

%% Nonlinear Rotation
% Angle-axis
syms a sa ca
ang = simplify(norm(r));
cosa = cos(ang);
sina = sin(ang);
u = simplify(r / ang);
ux = u(1);
uy = u(2);
uz = u(3);

% Rotation matrix
R = [        cosa + ux^2 * (1 - cosa), ux * uy * (1 - cosa) - uz * sina, ux * uz * (1 - cosa) + uy * sina;
     uy * ux * (1 - cosa) + uz * sina,         cosa + uy^2 * (1 - cosa), uy * uz * (1 - cosa) - ux * sina;
     uz * ux * (1 - cosa) - uy * sina, uz * uy * (1 - cosa) + ux * sina,         cosa + uz^2 * (1 - cosa)];
R_alt = cosa * eye(3) + sina*[0 -uz uy; uz 0 -ux; -uy ux 0 ] + (1 - cosa) * (u * u');
K = [0 -uz uy; uz 0 -ux; -uy ux 0];
R_alt2= eye(3) + sina*K + (1 - cosa) * K * K;

% Transform matrix
T = [R, t; 0 0 0 1];

%% Point-to-Point Nonlinear
f = simplify(R * p + t - q);
df_dx = simplify(jacobian(f, x));
dF_dx = simplify(2.0 * df_dx' * f);
d2F_dx2 = simplify(jacobian(dF_dx, x));
d2F_dzdx = simplify(jacobian(dF_dx, z));
d2F_dx2_unsimplified = d2F_dx2;
d2F_dzdx_unsimplified = d2F_dzdx;

% Simplify expressions
d2F_dx2 = subs(d2F_dx2, sina, sa);
d2F_dzdx = subs(d2F_dzdx, sina, sa);
d2F_dx2 = subs(d2F_dx2, cosa, ca);
d2F_dzdx = subs(d2F_dzdx, cosa, ca);
d2F_dx2 = subs(d2F_dx2, ang, a);
d2F_dzdx = subs(d2F_dzdx, ang, a);

% This achieves the same result, but is a larger matrix multiplication
f_alt = simplify(T * [p; 1] - [q; 1]);
df_dx_alt = simplify(jacobian(f_alt, x));
dF_dx_alt = simplify(2.0 * df_dx_alt' * f_alt);

% Save to file
fid = fopen("point_to_point_nonlinear.txt", "w");
for i = 1:length(d2F_dx2(1,:))
    for j = 1:length(d2F_dx2(:,1))
        fprintf(fid, "d2F_dx2(%d, %d) = %s;\n", i-1, j-1, char(d2F_dx2(i, j)));
    end
end
for i = 1:length(d2F_dzdx(1,:))
    for j = 1:length(d2F_dzdx(:,1))
        fprintf(fid, "d2F_dzdx(%d, %d) = %s;\n", i-1, j-1, char(d2F_dzdx(i, j)));
    end
end

%% Point-to-Plane Nonlinear
h = simplify(dot((R * p + t - q), n));
dh_dx = simplify(jacobian(h, x));
dH_dx = simplify(2.0 * dh_dx' * h);
d2H_dx2 = simplify(jacobian(dH_dx, x));
d2H_dzdx = simplify(jacobian(dH_dx, z));
d2H_dx2_unsimplified = d2H_dx2;
d2H_dzdx_unsimplified = d2H_dzdx;

% Simplify expressions
d2H_dx2 = subs(d2H_dx2, sina, sa);
d2H_dzdx = subs(d2H_dzdx, sina, sa);
d2H_dx2 = subs(d2H_dx2, cosa, ca);
d2H_dzdx = subs(d2H_dzdx, cosa, ca);
d2H_dx2 = subs(d2H_dx2, ang, a);
d2H_dzdx = subs(d2H_dzdx, ang, a);

% This achieves the same result, but is a larger matrix multiplication
h_alt = simplify(dot((T * [p; 1] - [q; 1]), [n; 1]));
dh_dx_alt = simplify(jacobian(h_alt, x));
dH_dx_alt = simplify(2.0 * dh_dx_alt' * h_alt);

% Save to file
fid = fopen("point_to_plane_nonlinear.txt", "w");
for i = 1:length(d2H_dx2(1,:))
    for j = 1:length(d2H_dx2(:,1))
        fprintf(fid, "d2H_dx2(%d, %d) = %s;\n", i-1, j-1, char(d2H_dx2(i, j)));
    end
end
for i = 1:length(d2H_dzdx(1,:))
    for j = 1:length(d2H_dzdx(:,1))
        fprintf(fid, "d2H_dzdx(%d, %d) = %s;\n", i-1, j-1, char(d2H_dzdx(i, j)));
    end
end

%% Generate tests for C++
vars = [rx, ry, rz, tx, ty, tz, px, py, pz, qx, qy, qz, nx, ny, nz];
values = [0.5, 0.9, 1.7, -4.5, 14.0, 8.9, 1.0, 2.0, 3.0, 5.5, 8.8, -4.0, ...
    0.455842305838552, 0.683763458757828, 0.569802882298190];
R_test = double(subs(R, vars, values));
f_test = double(subs(f, vars, values));
df_dx_test = double(subs(df_dx, vars, values));
dF_dx_test = double(subs(dF_dx, vars, values));

dH_dx_test = double(subs(dH_dx, vars, values));

d2F_dx2_test = double(subs(d2F_dx2_unsimplified, vars, values));
d2F_dzdx_test = double(subs(d2F_dzdx_unsimplified, vars, values));
d2F_dx2_lin_test = double(subs(d2F_dx2_lin, vars, values));
d2F_dzdx_lin_test = double(subs(d2F_dzdx_lin, vars, values));

d2H_dx2_test = double(subs(d2H_dx2_unsimplified, vars, values));
d2H_dzdx_test = double(subs(d2H_dzdx_unsimplified, vars, values));
d2H_dx2_lin_test = double(subs(d2H_dx2_lin, vars, values));
d2H_dzdx_lin_test = double(subs(d2H_dzdx_lin, vars, values));
