% Code Description: Stability and Control Coefficient Derivatives Integrator.
% Author: Matheus Rocha Carlos
% Email: ROCHACAM@my.erau.edu
% Class: AE413 - Section 02DB
% Date: 04/21/2026
% Worked With: N/a
% Unites: per deg, degree
clear; clc; close all;
% Data Preparation
alpha = -8:2:20;
CLB_values = [-7.66E-04, -8.01E-04, -8.34E-04, -8.63E-04, -8.90E-04,-9.21E-04, -9.55E-04, -9.90E-04, -1.03E-03, -1.07E-03, -1.10E-03, -1.12E-03, -1.13E-03, -1.13E-03, -1.08E-03];
CLA_values = [1.05E-01, 1.01E-01, 9.83E-02, 9.64E-02, 9.80E-02 ,1.02E-01, 1.05E-01, 1.06E-01, 1.08E-01, 1.09E-01, 1.01E-01, 8.73E-02, 7.44E-02, 4.67E-02, 4.83E-03];
CYP_values = [-8.25E-03, -7.96E-03, -7.65E-03, -7.45E-03, -7.61E-03, -7.95E-03, -8.22E-03, -8.43E-03, -8.57E-03, -8.14E-03, -6.81E-03, -5.26E-03, -3.74E-03, -7.39E-04, 3.64E-03];
CNR_values = [1.09E-03, -1.13E-03, -1.17E-03, -1.20E-03, -1.22E-03, -1.23E-03, -1.23E-03, -1.23E-03, -1.22E-03, -1.20E-03, -1.17E-03, -1.15E-03, -1.12E-03, -1.11E-03, -1.11E-03];
CLR_values = [-1.73E-03, -1.02E-03, -3.37E-04, 3.12E-04, 9.57E-04, 1.63E-03, 2.34E-03, 3.06E-03, 3.80E-03, 4.55E-03, 5.20E-03, 5.69E-03, 6.06E-03, 6.26E-03, 6.07E-03];
% Calculate f(alpha) = value * alpha
f_CLB_alpha = CLB_values .* alpha;
f_CLA_alpha = CLA_values .* alpha;
f_CYP_alpha = CYP_values .* alpha;
f_CNR_alpha = CNR_values .* alpha;
f_CLR_alpha = CLR_values .* alpha;
% Calculate area using trapz
integral_CLB = trapz(alpha, f_CLB_alpha);
fprintf('The CLB integrated value is: %.5f\n', integral_CLB);
integral_CLA = trapz(alpha, f_CLA_alpha);
fprintf('The CLA integrated value is: %.5f\n', integral_CLA);
integral_CYP = trapz(alpha, f_CYP_alpha);
fprintf('The CYP integrated value is: %.5f\n', integral_CYP);
integral_CNR = trapz(alpha, f_CNR_alpha);
fprintf('The CNR integrated value is: %.5f\n', integral_CNR);
integral_CLR = trapz(alpha, f_CLR_alpha);
fprintf('The CLR integrated value is: %.5f\n', integral_CLR);