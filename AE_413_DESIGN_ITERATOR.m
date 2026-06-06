% Code Name: Aircraft Geometry Iterator for Stability and Control Analysis
% Code Description: Hard coded iterator for for Stability and Control Analysis of a general aviation aircraft.
% Author: Matheus Rocha Carlos
% Email: ROCHACAM@my.erau.edu
% Class: AE413 - Section 02DB
% Date: 04/06/2026 until 04/20/2026
% Worked With: N/a | Utilized Copilot(Ctrl+Shift+P) and ClaudeAi for general iterator built based on baseline Cessna 208B design inputs. 
% Equations and calculations for error check taken from: http://wpage.unina.it/danilo.ciliberti/doc/Giorgio.pdf
%  Reference frame : nose = x=0, fuselage centreline = z=0
%  Units           : ft, lb, slug, rad (angles converted internally)

clc; clear; close all;

%% DESIGN INPUTS

% Flight Condition Input
h_ft        = 5000;          % [ft]   analysis altitude
M_cruise    = 0.20;          % [-]    cruise Mach number
alpha_fus   = 0.0;           % [deg]  fuselage angle of attack

% Main Wing Inputs
wing_airfoil = 2412;
b_w         = 36.0;          % [ft]   wingspan
S_w         = 160.0;         % [ft²]  planform area
lambda_w    = 0.40;          % [-]    taper ratio
Lambda_LE_w = 5.0;           % [deg]  leading-edge sweep
Gamma_w     = 2.0;           % [deg]  dihedral
TC_w        = 0.12;          % [-]    thickness ratio (NACA 2412)
ALIW        = 2.0;           % [deg]  wing root incidence
x_wLE       = 11.0;          % [ft]   wing root LE from nose
ZW          = -1.20;         % [ft]   wing apex vertical location (altered)

% Horizontal Tail Inputs
ht_airfoil = 0009;
b_H         = 12.0;          % [ft]   HT span
S_H         = 35.0;          % [ft²]  HT planform area
lambda_H    = 0.50;          % [-]    HT taper ratio
Lambda_LE_H = 8.0;           % [deg]  HT LE sweep
ALIH        = -2.0;          % [deg]  HT root incidence
ZH          = 0.0;           % [ft]   HT apex vertical location
x_HLE       = 26.0;          % [ft]   HT root LE from nose
TC_H        = 0.09;          % [-]    airfoil thickness

% Vertical Tail Inputs
vt_airfoil = 0009;
b_V         = 5.0;           % [ft]   VT span
S_V         = 15.0;          % [ft²]  VT planform area
lambda_V    = 0.40;          % [-]    VT taper ratio
Lambda_LE_V = 25.0;          % [deg]  VT LE sweep
ZV          = 0.86;          % [ft]   VT apex vertical location (altered)
x_VLE       = 25.5;          % [ft]   VT root LE from nose

% Fuselage Inputs
lf          = 30.0;          % [ft]   fuselage total length
d_max       = 4.50;          % [ft]   maximum fuselage diameter
r_max       = d_max / 2;     % [ft]   maximum radius
ZCG_z       = -0.50;         % [ft]   CG vertical location

% Mass and Thrust Inputs
m_lb        = 3400;          % [lb]   total mass
T_max_N     = 4000;          % [N]    maximum thrust
n_pax       = 6;             % [-]    total occupants (pilot + 5)

% Center of Gravity Location Input
x_CG        = 13.09;         % [ft]   CG from nose

%% DERIVED WING GEOMETRY
AR_w        = b_w^2 / S_w;
cr_w        = 2*S_w / (b_w*(1 + lambda_w));
ct_w        = lambda_w * cr_w;
mac_w       = cr_w * (2/3) * (1 + lambda_w + lambda_w^2) / (1 + lambda_w);
y_mac       = (b_w/6) * (1 + 2*lambda_w) / (1 + lambda_w);
tanLE_w     = tand(Lambda_LE_w);
tanC4_w     = tanLE_w - (1/AR_w)*(1 - lambda_w)/(1 + lambda_w);
tanC2_w     = tanLE_w - (2/AR_w)*(1 - lambda_w)/(1 + lambda_w);
Lambda_C4_w = atand(tanC4_w);
x_AC_w      = x_wLE + cr_w/4 + tanC4_w * y_mac;   
x_MALE      = x_wLE + tanLE_w * y_mac;             
x_wTE       = x_wLE + cr_w;                        
lf_prime    = x_wLE + 0.5*cr_w;                   
fprintf('  Aspect ratio          AR  = %.3f\n', AR_w);
fprintf('  Root chord            cr  = %.3f ft\n', cr_w);
fprintf('  Tip chord             ct  = %.3f ft\n', ct_w);
fprintf('  Mean aero chord       MAC = %.3f ft\n', mac_w);
fprintf('  MAC LE from nose          = %.3f ft\n', x_MALE);
fprintf('  Wing AC from nose         = %.3f ft\n', x_AC_w);
fprintf('  Quarter-chord sweep   Lc4 = %.2f deg\n', Lambda_C4_w);
fprintf('  Half-chord ref (lf'')      = %.3f ft\n\n', lf_prime);

%% DERIVED TAIL GEOMETRY

% Horizontal tail derived geometries
cr_H        = 2*S_H / (b_H*(1 + lambda_H));
ct_H        = lambda_H * cr_H;
mac_H       = cr_H * (2/3)*(1 + lambda_H + lambda_H^2)/(1 + lambda_H);
AR_H        = b_H^2 / S_H;
tanC4_H     = tand(Lambda_LE_H) - (1/AR_H)*(1 - lambda_H)/(1 + lambda_H);
Lambda_C4_H = atand(tanC4_H);
y_mac_H     = (b_H/6)*(1 + 2*lambda_H)/(1 + lambda_H);
x_AC_H      = x_HLE + cr_H/4 + tanC4_H * y_mac_H;
l_H         = x_AC_H - x_AC_w;                 

% Vertical tail derived geometries
cr_V        = 2*S_V / (b_V*(1 + lambda_V));
ct_V        = lambda_V * cr_V;
mac_V       = cr_V * (2/3)*(1 + lambda_V + lambda_V^2)/(1 + lambda_V);
AR_V        = b_V^2 / S_V;
tanC4_V     = tand(Lambda_LE_V) - (1/AR_V)*(1 - lambda_V)/(1 + lambda_V);
Lambda_C4_V = atand(tanC4_V);
y_mac_V     = (b_V/6)*(1 + 2*lambda_V)/(1 + lambda_V);
x_AC_V      = x_VLE + cr_V/4 + tanC4_V * y_mac_V;
l_V         = x_AC_V - x_CG;                    

% Tail volume ratios
V_H_coeff   = (S_H * l_H) / (S_w * mac_w);
V_V_coeff   = (S_V * l_V) / (S_w * b_w);
fprintf('  HT: cr=%.3f ft  ct=%.3f ft  MAC=%.3f ft  AR=%.3f\n', cr_H, ct_H, mac_H, AR_H);
fprintf('  HT: AC from nose = %.3f ft  | Moment arm lH = %.3f ft\n', x_AC_H, l_H);
fprintf('  HT: Tail volume VH = %.4f\n', V_H_coeff);
fprintf('  VT: cr=%.3f ft  ct=%.3f ft  MAC=%.3f ft  AR=%.3f\n', cr_V, ct_V, mac_V, AR_V);
fprintf('  VT: AC from nose = %.3f ft  | Moment arm lV = %.3f ft\n', x_AC_V, l_V);
fprintf('  VT: Tail volume VV = %.4f\n\n', V_V_coeff);

%% ISA ATMOSPHERE AT ALTITUDE h_ft
% Input Values
T_R         = 518.67 - 3.5662 * (h_ft/1000);      % temperature [°R]
T_K         = T_R / 1.8;                           % [K]
rho_0       = 0.002377;                            % sea-level density [slug/ft³]
T_0_R       = 518.67;
rho         = rho_0 * (T_R/T_0_R)^4.256;          % density [slug/ft³]
a_ft        = 49.021 * sqrt(T_R);                 % speed of sound [ft/s]
V_inf       = M_cruise * a_ft;                     % TAS [ft/s]
q_inf       = 0.5 * rho * V_inf^2;                % dynamic pressure [lb/ft²]
mu          = 3.637e-7 * (T_R/518.67)^1.5 * (518.67 + 198.6)/(T_R + 198.6); % Sutherland viscosity
nu          = mu / rho;                            % kinematic viscosity [ft²/s]
beta_M      = sqrt(1 - M_cruise^2);               % Prandtl-Glauert factor
fprintf('  Temperature       T   = %.2f °R  (%.2f K)\n', T_R, T_K);
fprintf('  Density           rho = %.6f slug/ft³\n', rho);
fprintf('  Speed of sound    a   = %.2f ft/s\n', a_ft);
fprintf('  True airspeed     V   = %.2f ft/s\n', V_inf);
fprintf('  Dynamic pressure  q∞  = %.3f lb/ft²\n', q_inf);
fprintf('  Kin. viscosity    nu  = %.4e ft²/s\n', nu);
fprintf('  P-G factor        beta= %.4f\n\n', beta_M);


%% AERODYNAMIC COEFFICIENTS (NACA 2412 MAIN WING)

% 2D NACA 2412 airfoil knwon data from NASA database 
CL0_2D      = 0.25;  
CD0_2D      = 0.007;  
CLalpha_2D  = 6.30;   

% Finite wing CLalpha per rad
tanC2_w_b   = tanC2_w / beta_M;
denom_w     = 2 + sqrt(4 + (AR_w*beta_M)^2 * (1 + tanC2_w_b^2));
CLalpha_w   = (2*pi*AR_w*beta_M) / denom_w;      

% Wing CL at cruise angle of attack (ALIW + alpha_fus)
alpha_wing_rad = deg2rad(alpha_fus + ALIW);
CL_wing     = CL0_2D + CLalpha_w * alpha_wing_rad;

% Wing induced drag per rad
e_w         = 0.85;   % Assumed Oswald efficiency
CD_induced  = CL_wing^2 / (pi * AR_w * e_w);
CD_total_w  = CD0_2D + CD_induced;

% Downwash gradient at HT
de_da       = 2 * CLalpha_w / (pi * AR_w);    
fprintf('  NACA 2412 section:  CL0 = %.3f | CD0 = %.4f | CLa_2D = %.3f /rad\n', ...
        CL0_2D, CD0_2D, CLalpha_2D);
fprintf('  3D Wing CLalpha     = %.4f /rad  (Helmbold, AR=%.2f, M=%.2f)\n', ...
        CLalpha_w, AR_w, M_cruise);
fprintf('  Wing CL at cruise   = %.4f  (alpha_w = %.2f deg)\n', ...
        CL_wing, alpha_fus + ALIW);
fprintf('  CD_total (wing)     = %.5f\n', CD_total_w);
fprintf('  Downwash gradient   de/da = %.4f\n\n', de_da);


%% DYNAMIC PRESSURE RATIOS

BL_thickness = @(x) 0.37 * x / (V_inf * x / nu)^0.2;
eta_BL_avg  = @(delta, b_semi)(delta * (7/9) + (b_semi - delta)) / b_semi;
% Horizontal tail dynamic pressure ratio
x_HT        = x_HLE;                             
Re_H        = V_inf * x_HT / nu;
delta_H     = BL_thickness(x_HT);
eta_H_BL    = eta_BL_avg(delta_H, b_H/2);
% Wing wake deficit at HT
x_wake_H    = x_AC_H - x_AC_w;                 
delta_eta_wake = CD_total_w * sqrt(mac_w / (4*pi*x_wake_H));
eta_H       = eta_H_BL - delta_eta_wake;
q_H         = eta_H * q_inf;
fprintf('  HORIZONTAL TAIL:\n');
fprintf('    BL run length          x_H  = %.2f ft\n', x_HT);
fprintf('    Reynolds number        Re_H = %.4e\n', Re_H);
fprintf('    BL thickness           d_H  = %.4f ft\n', delta_H);
fprintf('    BL eta (averaged)            = %.4f\n', eta_H_BL);
fprintf('    Wing wake deficit     Deta  = %.4f\n', delta_eta_wake);
fprintf('    Combined eta_H               = %.4f\n', eta_H);
fprintf('    Dynamic pressure      q_H   = %.3f lb/ft²\n\n', q_H);
% Vertical tail dynamic pressure ratio
x_VT        = x_VLE;
Re_V        = V_inf * x_VT / nu;
delta_V     = BL_thickness(x_VT);
eta_V_BL    = eta_BL_avg(delta_V, b_V);          
z_sep       = abs(ZV - ZCG_z);                   
wake_width  = 0.2 * x_wake_H;                   
delta_eta_VT_wake = delta_eta_wake * exp(-0.5*(z_sep/wake_width)^2);
eta_V       = eta_V_BL - delta_eta_VT_wake;
q_V         = eta_V * q_inf;
fprintf('  VERTICAL TAIL:\n');
fprintf('    BL run length          x_V  = %.2f ft\n', x_VT);
fprintf('    Reynolds number        Re_V = %.4e\n', Re_V);
fprintf('    BL thickness           d_V  = %.4f ft\n', delta_V);
fprintf('    BL eta (averaged)            = %.4f\n', eta_V_BL);
fprintf('    Wing wake deficit (VT)       = %.4f  (dorsal, attenuated)\n', delta_eta_VT_wake);
fprintf('    Combined eta_V               = %.4f\n', eta_V);
fprintf('    Dynamic pressure      q_V   = %.3f lb/ft²\n\n', q_V);


%% 8 STABILITY DERIVATIVES ANALYSIS
% Tail lift slopes
tanC2_H     = tand(Lambda_LE_H) - (2/AR_H)*(1 - lambda_H)/(1 + lambda_H);
tanC2_H_b   = tanC2_H / beta_M;
denom_H     = 2 + sqrt(4 + (AR_H*beta_M)^2*(1 + tanC2_H_b^2));
CLalpha_H   = (2*pi*AR_H*beta_M) / denom_H;
tanC2_V     = tand(Lambda_LE_V) - (2/AR_V)*(1 - lambda_V)/(1 + lambda_V);
tanC2_V_b   = tanC2_V / beta_M;
denom_V     = 2 + sqrt(4 + (AR_V*beta_M)^2*(1 + tanC2_V_b^2));
CLalpha_V   = (2*pi*AR_V*beta_M) / denom_V;
%Neutral point
K_H         = eta_H * (CLalpha_H / CLalpha_w) * (1 - de_da);
x_NP        = x_AC_w + (S_H/S_w) * l_H * K_H;
% Static margin and Cmα
SM          = (x_NP - x_CG) / mac_w;
CLalpha_tot = CLalpha_w * (1 + eta_H * (S_H/S_w) * (CLalpha_H/CLalpha_w) * (1 - de_da));
Cmalpha     = -CLalpha_tot * SM;
% Directional stability c_n,beta
bV_eff      = b_V - ZV;                          
SV_eff      = S_V * (bV_eff / b_V)^2;            
Cnbeta_VT   = eta_V * (SV_eff/S_w) * (l_V/b_w) * CLalpha_V;
Cnbeta_fus  = -0.015;                          
Cnbeta_tot  = Cnbeta_VT + Cnbeta_fus;
fprintf('  Tail lift slopes:\n');
fprintf('    CLalpha_H = %.4f /rad  (AR=%.2f)\n', CLalpha_H, AR_H);
fprintf('    CLalpha_V = %.4f /rad  (AR=%.2f)\n', CLalpha_V, AR_V);
fprintf('  Neutral point:\n');
fprintf('    x_NP      = %.4f ft from nose\n', x_NP);
fprintf('    NP/MAC    = %.3f (%.1f%% MAC)\n', x_NP/mac_w, (x_NP-x_wLE)/mac_w*100);
fprintf('  Longitudinal:\n');
fprintf('    CLalpha_total  = %.4f /rad\n', CLalpha_tot);
fprintf('    Static margin  = %.4f (%.1f%% MAC)\n', SM, SM*100);
fprintf('    Cmalpha        = %.4f /rad\n', Cmalpha);
fprintf('  Directional:\n');
fprintf('    CLalpha_V      = %.4f /rad\n', CLalpha_V);
fprintf('    bV_eff         = %.4f ft  (after ZV correction)\n', bV_eff);
fprintf('    Cnbeta_VT      = %+.5f /rad\n', Cnbeta_VT);
fprintf('    Cnbeta_fus     = %+.5f /rad\n', Cnbeta_fus);
fprintf('    Cnbeta_total   = %+.5f /rad\n\n', Cnbeta_tot);

%%  8 FUSELAGE CROSS-SECTION MODEL WITH 10-STATION BODY DIVISION
x_rmax      = 14.0;   % assumed location of maximum fuselage radius
ns_fore     = 4;      % number of before wing LE stations
ns_aft      = 5;      % number of after wing TE stations
x_fore_bounds = linspace(0, x_wLE, ns_fore+1);         
x_aft_bounds  = linspace(x_wTE, lf, ns_aft+1);       
Xi_fore = 0.5*(x_fore_bounds(1:end-1) + x_fore_bounds(2:end));
Xi_wing = 0.5*(x_wLE + x_wTE);                        
Xi_aft  = 0.5*(x_aft_bounds(1:end-1)  + x_aft_bounds(2:end));
Xi_all  = [Xi_fore, Xi_wing, Xi_aft];
% Fuselage radius function calculator
r_fus = @(x) ...
    (x <= x_rmax) .* (r_max * (max(x,0.001)/x_rmax).^0.55) + ...
    (x >  x_rmax) .* (r_max * max(1 - 0.85*((x - x_rmax)/(lf - x_rmax)).^1.2, 0));
ZU_correction = zeros(1,10);
ZU_correction(10) = 0.850 - r_fus(Xi_all(10));  
ZU_all = arrayfun(r_fus, Xi_all) + ZU_correction; 
ZL_all = -arrayfun(r_fus, Xi_all);                  
R_all  = ZU_all - ZL_all;                            
% Cross-sectional area assumed ellipse with semi-axes ZU and r_fus
S_cs   = pi * arrayfun(r_fus, Xi_all) .* (R_all/2);
stn_type = {'Fore','Fore','Fore','Fore','Wing','Aft','Aft','Aft','Aft','Aft'};
fprintf('\n  %-4s %-10s %-10s %-10s %-10s %-10s %-8s\n', ...
    'Stn','Xi (ft)','ZU (ft)','ZL (ft)','R (ft)','S (ft²)','Type');
fprintf('  %s\n', repmat('-',1,64));
for i = 1:10
    fprintf('  %-4d %-10.4f %-10.4f %-10.4f %-10.4f %-10.4f %-8s\n', ...
        i, Xi_all(i), ZU_all(i), ZL_all(i), R_all(i), S_cs(i), stn_type{i});
end
% Additional fuselage parameters
r1      = mean(arrayfun(r_fus, [x_VLE, 0.5*(x_VLE+lf), lf]));   
SFS     = trapz([0, Xi_all, lf], [0, R_all, 0]);                  
d_wavg  = 2 * mean(arrayfun(r_fus, linspace(x_wLE, x_wTE, 20)));  
fprintf('\n  Fuselage summary:\n');
fprintf('    r1 (avg radius under VT)     = %.4f ft\n', r1);
fprintf('    SFS (projected side area)    = %.3f ft²\n', SFS);
fprintf('    d_wavg (avg diam at wing)    = %.4f ft\n\n', d_wavg);

%%  REQUIREMENTS CHECKING SYSTEM
pass_sym  = '  [PASS]';
fail_sym  = '  [FAIL]';
n_pass    = 0;
n_fail    = 0;
function result = check_req(label, value, limit_lo, limit_hi, unit, pass_n, fail_n)
    if ~isempty(limit_lo) && ~isempty(limit_hi)
        ok = (value >= limit_lo) && (value <= limit_hi);
        lim_str = sprintf('%.3g < value < %.3g %s', limit_lo, limit_hi, unit);
    elseif isempty(limit_lo)
        ok = value < limit_hi;
        lim_str = sprintf('value < %.4g %s', limit_hi, unit);
    else
        ok = value > limit_lo;
        lim_str = sprintf('value > %.4g %s', limit_lo, unit);
    end
    if ok
        tag = '  [PASS]';
        pass_n = pass_n + 1;
    else
        tag = '  [FAIL]';
        fail_n = fail_n + 1;
    end
    fprintf('%s  %-40s  value = %9.4f %-6s  (%s)\n', tag, label, value, unit, lim_str);
    result = ok;
end

% Geometric requirements
fprintf('  GEOMETRIC REQUIREMENTS\n');
fprintf('  %s\n', repmat('-',1,80));
r1b = check_req('Wing area S',           S_w,        120,   200,    'ft²',  n_pass, n_fail); n_pass=n_pass+r1b; n_fail=n_fail+~r1b;
r2b = check_req('Wingspan b',            b_w,         20,    50,    'ft',   n_pass, n_fail); n_pass=n_pass+r2b; n_fail=n_fail+~r2b;
r3b = check_req('Mean aero chord MAC',   mac_w,        4,     6,    'ft',   n_pass, n_fail); n_pass=n_pass+r3b; n_fail=n_fail+~r3b;
r4b = check_req('Total mass m',          m_lb,         0,  3500,    'lb',   n_pass, n_fail); n_pass=n_pass+r4b; n_fail=n_fail+~r4b;
r5b = check_req('Fuselage length lf',    lf,           0,    35,    'ft',   n_pass, n_fail); n_pass=n_pass+r5b; n_fail=n_fail+~r5b;
r6b = check_req('Wing LE sweep Lambda',  Lambda_LE_w,  3,   Inf,    'deg',  n_pass, n_fail); n_pass=n_pass+r6b; n_fail=n_fail+~r6b;
r7a = (Gamma_w>1 && Gamma_w<3) || (Gamma_w>-3 && Gamma_w<-1);
if r7a; tag='  [PASS]'; n_pass=n_pass+1; else; tag='  [FAIL]'; n_fail=n_fail+1; end
fprintf('%s  %-40s  value = %9.4f %-6s  (1<|G|<3 deg)\n', tag, 'Dihedral Gamma', Gamma_w, 'deg');
r8b = check_req('Max thrust T',          T_max_N,   3000,  5000,    'N',    n_pass, n_fail); n_pass=n_pass+r8b; n_fail=n_fail+~r8b;
r9b = (n_pax >= 6);
if r9b; tag='  [PASS]'; n_pass=n_pass+1; else; tag='  [FAIL]'; n_fail=n_fail+1; end
fprintf('%s  %-40s  value = %9d %-6s  (>= 6)\n', tag, 'Occupants', n_pax, 'seats');
fprintf('\n  AERODYNAMIC REQUIREMENTS\n');
fprintf('  %s\n', repmat('-',1,80));
ra1 = check_req('CL0 section (NACA 2412)',   CL0_2D,        -Inf,  0.40,  '/rad', n_pass, n_fail); n_pass=n_pass+ra1; n_fail=n_fail+~ra1;
ra2 = check_req('CD0 section (NACA 2412)',   CD0_2D,        -Inf,  0.060, '[-]',  n_pass, n_fail); n_pass=n_pass+ra2; n_fail=n_fail+~ra2;
ra3 = check_req('CLalpha (3D wing)',         CLalpha_w,     -Inf,  6.00,  '/rad', n_pass, n_fail); n_pass=n_pass+ra3; n_fail=n_fail+~ra3;
ra4 = check_req('Cmalpha (pitch stability)', Cmalpha,       -Inf, -0.90,  '/rad', n_pass, n_fail); n_pass=n_pass+ra4; n_fail=n_fail+~ra4;
ra5 = check_req('Cnbeta (dir. stability)',   Cnbeta_tot,    -Inf,  0.08,  '/rad', n_pass, n_fail); n_pass=n_pass+ra5; n_fail=n_fail+~ra5;
fprintf('\n  FLIGHT ENVELOPE\n');
fprintf('  %s\n', repmat('-',1,80));
rf1 = check_req('Mach number',    M_cruise, 0.10, 0.40, '[-]', n_pass, n_fail); n_pass=n_pass+rf1; n_fail=n_fail+~rf1;
rf2 = check_req('Altitude',       h_ft,    2000, 7000, 'ft',  n_pass, n_fail); n_pass=n_pass+rf2; n_fail=n_fail+~rf2;
fprintf('\n  STABILITY MARGIN CHECKS\n');
fprintf('  %s\n', repmat('-',1,80));
rs1 = check_req('Static margin SM',      SM,      0.05, 0.35, '[-]',  n_pass, n_fail); n_pass=n_pass+rs1; n_fail=n_fail+~rs1;
rs2 = check_req('HT tail volume VH',     V_H_coeff, 0.30, 1.20, '[-]',  n_pass, n_fail); n_pass=n_pass+rs2; n_fail=n_fail+~rs2;
rs3 = check_req('VT tail volume VV',     V_V_coeff, 0.02, 0.10, '[-]',  n_pass, n_fail); n_pass=n_pass+rs3; n_fail=n_fail+~rs3;

fprintf('\n=================================================================\n');
fprintf('  SUMMARY: %d PASSED | %d FAILED | %d TOTAL\n', ...
    n_pass, n_fail, n_pass+n_fail);
if n_fail == 0
    fprintf('  >> ALL REQUIREMENTS SATISFIED\n');
else
    fprintf('  >> DESIGN REQUIRES REVISION — see [FAIL] items above\n');
end
fprintf('=================================================================\n\n');

%%  GEOMETRIC PLOTS
% Plot 1: Wing planform top view
fig1 = figure('Name','Wing Planform','Position',[50 50 700 500]);
hold on; axis equal; grid on; grid minor;
title('GA-6 Wing Planform (Top View)','FontSize',14,'FontWeight','bold');
xlabel('Longitudinal station from nose (ft)'); ylabel('Spanwise (ft)');
tanLE = tand(Lambda_LE_w);
x_root_LE=x_wLE; x_root_TE=x_wLE+cr_w;
x_tip_LE =x_wLE + tanLE*(b_w/2);
x_tip_TE =x_tip_LE + ct_w;
Xw=[x_root_LE x_root_TE x_tip_TE x_tip_LE x_root_LE];
Yw=[0 0 b_w/2 b_w/2 0];
fill(Xw, Yw, [0.68 0.85 0.90],'EdgeColor',[0.1 0.4 0.6],'LineWidth',1.2);
fill(Xw,-Yw,[0.68 0.85 0.90],'EdgeColor',[0.1 0.4 0.6],'LineWidth',1.2);
theta_f=linspace(0,2*pi,200);
plot(15+15*cos(theta_f), 2.25*sin(theta_f),'c-','LineWidth',1.2);
tanLE_h=tand(Lambda_LE_H);
xhr_LE=x_HLE; xhr_TE=x_HLE+cr_H;
xht_LE=x_HLE+tanLE_h*(b_H/2); xht_TE=xht_LE+ct_H;
XH=[xhr_LE xhr_TE xht_TE xht_LE xhr_LE];
YH=[0 0 b_H/2 b_H/2 0];
fill(XH,YH,[0.75 0.92 0.82],'EdgeColor',[0.1 0.55 0.35],'LineWidth',1.2);
fill(XH,-YH,[0.75 0.92 0.82],'EdgeColor',[0.1 0.55 0.35],'LineWidth',1.2);
fill([x_VLE x_VLE+cr_V x_VLE+cr_V x_VLE x_VLE],...
     [0 0 0.4 0.4 0],...
     [0.9 0.82 0.65],'EdgeColor',[0.6 0.4 0.1],'LineWidth',1.2);
plot(x_CG, 0,'ro','MarkerSize',8,'MarkerFaceColor','r');
plot(x_NP, 0,'bs','MarkerSize',8,'MarkerFaceColor','b');
xline(x_CG,'r--','LineWidth',0.8);
xline(x_NP,'b--','LineWidth',0.8);
plot([x_MALE, x_MALE+mac_w],[y_mac y_mac],'m-','LineWidth',2);
plot([x_MALE, x_MALE+mac_w],[-y_mac -y_mac],'m-','LineWidth',2);
legend({'Wing (port)','Wing (stbd)','Fuselage','HT (port)','HT (stbd)',...
        'VT (projected)','CG','Neutral point','MAC'},...
    'Location','NorthWest','FontSize',9);
xlim([0 31]); ylim([-b_w/2-2 b_w/2+2]);

%% GEOMETRY SUMMARY TABLE
fprintf('  GEOMETRY SUMMARY TABLE\n');
fprintf('\n  %-35s %-15s %-15s %-15s\n','Parameter','Main Wing','Horiz. Tail','Vert. Tail');
fprintf('  %s\n', repmat('-',1,82));
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Span (ft)',        b_w,    b_H,    b_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Area (ft²)',       S_w,    S_H,    S_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Aspect ratio',     AR_w,   AR_H,   AR_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Taper ratio',      lambda_w,lambda_H,lambda_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Root chord (ft)',  cr_w,   cr_H,   cr_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Tip chord (ft)',   ct_w,   ct_H,   ct_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','MAC (ft)',         mac_w,  mac_H,  mac_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','LE sweep (deg)',   Lambda_LE_w,Lambda_LE_H,Lambda_LE_V);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','c/4 sweep (deg)', Lambda_C4_w,Lambda_C4_H,Lambda_C4_V);
fprintf('  %-35s %-15.3f %-15s %-15s\n',    'Dihedral (deg)',   Gamma_w,'0.0','90.0 (DATCOM)');
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','Root LE from nose',x_wLE,  x_HLE,  x_VLE);
fprintf('  %-35s %-15.3f %-15.3f %-15.3f\n','AC from nose (ft)',x_AC_w, x_AC_H, x_AC_V);
fprintf('\n  %-35s %-15.3f\n','CG from nose (ft)',   x_CG);
fprintf('  %-35s %-15.3f\n','NP from nose (ft)',   x_NP);
fprintf('  %-35s %-15.3f (%.1f%% MAC)\n','Static margin', SM, SM*100);
fprintf('  %-35s %-15.3f ft\n','Fuselage length lf',  lf);
fprintf('  %-35s %-15.3f ft\n','Max fuselage diameter',d_max);
fprintf('  %-35s %-15.3f ft\n','ZW  (wing apex Z)',   ZW);
fprintf('  %-35s %-15.3f ft\n','ZV  (VT apex Z)',     ZV);
fprintf('  %-35s %-15.3f ft\n','ZCG (CG vertical)',   ZCG_z);
fprintf('\n  %-35s %-15.4f\n','eta_H (q_H/q∞)', eta_H);
fprintf('  %-35s %-15.4f\n','eta_V (q_V/q∞)', eta_V);
fprintf('  %-35s %-15.4f /rad\n','CLalpha (3D wing)',   CLalpha_w);
fprintf('  %-35s %-15.4f /rad\n','Cmalpha',             Cmalpha);
fprintf('  %-35s %-15.5f /rad\n','Cnbeta_total',        Cnbeta_tot);
