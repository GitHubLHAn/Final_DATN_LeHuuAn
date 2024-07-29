
% Rf = 0.195; Lf = 0.7e-3; Rc = 0.1; Cf = 120e-6; Vd = 250;
Fsw = 10000; Ts = 1/Fsw;

Vd = 400;             %Vdc
% Omega_sw = 2*pi*Fsw;    %rad/s
% Ts = 1/Fsw;             %s
% Vac_rms = 220;          %V
% Irms = 25;              %A
% Iripple = 0.3;          %30%

% Lf = Vd/(4*Fsw*Irms*sqrt(2)*Iripple); %H
% Cf = 100/(Omega_sw^2*Lf);

Lf = 0.4e-3; Cf = 22e-6; 
Rf = 0.05; Rc = 0.05;

% Lf = 0.423e-3+0.429e-3; Cf = 20e-6; 
% Rf = 0.01; Rc = 0.05;

A = [-Rc/Lf (1/Cf-Rc*Rf/Lf);
    -1/Lf -Rf/Lf]
B = [Rc*Vd/Lf;
     Vd/Lf]
C = [1 0; 
     0 1]
[Ad,Bd] = c2d(A,B,Ts)

M = [Ad eye(2,2);
    zeros(2,2) eye(2,2)]
G = [1 0 0 0;
      0 1 0 0]
p = [0.2 0.1 0.15 0.023];
L = place(M',G',p);
L = L'

% Lx = 0.1e-3; Cx = 50e-6; 
% Omegaf = 1/sqrt(Lf*Cf)
% Omegax = 1/sqrt(Lx*Cx)
% OmegaswD10 = 2*pi*Fsw/10
% DeltaIppMax_Lx = Vd*Ts/(4*Lx)
% DeltaIppMax_Lf = Vd*Ts/(4*Lf)
