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
% point normal
nx = sym('nx', 'real');
ny = sym('ny', 'real');
nz = sym('nz', 'real');
n = [nx; ny; nz];

%% Rotation
% Angle-axis
a = simplify(norm(r));
cosa = cos(a);
sina = sin(a);
u = simplify(r / a);
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

%% Point-to-Point
G = simplify(R * p + t - q);
dG_dx = simplify(jacobian(G, x));
df_dx = simplify(2.0 * dG_dx' * G);

% This achieves the same result, but is a larger matrix multiplication
G_alt = simplify(T * [p; 1] - [q; 1]);
dG_dx_alt = simplify(jacobian(G_alt, x));
df_dx_alt = simplify(2.0 * dG_dx_alt' * G_alt);

%% Point-to-Plane
H = simplify(dot((R * p + t - q), n));
dH_dx = simplify(jacobian(H, x));
dg_dx = simplify(2.0 * dH_dx' * H);

% This achieves the same result, but is a larger matrix multiplication
H_alt = simplify(dot((T * [p; 1] - [q; 1]), [n; 1]));
dH_dx_alt = simplify(jacobian(H_alt, x));
dg_dx_alt = simplify(2.0 * dH_dx_alt' * H_alt);

%% Generate tests for C++
vars = [rx, ry, rz, tx, ty, tz, px, py, pz, qx, qy, qz, nx, ny, nz];
values = [0.5, 0.9, 1.7, -4.5, 14.0, 8.9, 1.0, 2.0, 3.0, 5.5, 8.8, -4.0, ...
    0.455842305838552, 0.683763458757828, 0.569802882298190];
R_test = double(subs(R, vars, values));
G_test = double(subs(G, vars, values));
dG_dx_test = double(subs(dG_dx, vars, values));
df_dx_test = double(subs(df_dx, vars, values));

dg_dx_test = double(subs(dg_dx, vars, values));
