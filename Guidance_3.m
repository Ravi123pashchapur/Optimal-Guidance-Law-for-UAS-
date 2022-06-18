function [AM ZEM] = Guidance_3(R1, SIG1, GAM1, SIGR1)
global GAMD VM1
    
%.. Guidance Law
    % Parameters
    N = 5;
    Tgo = R1/VM1;
    R_dot = -VM1*cos(GAMD-SIG1);
    
    y = R1*(GAMD-SIG1);
    v = VM1*(GAM1-GAMD);
    
    % Guidance Command
   AM = (-N/Tgo^2)*(y+v*Tgo);
   ZEM = (y+v*Tgo);