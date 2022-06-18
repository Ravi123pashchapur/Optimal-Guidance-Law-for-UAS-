function [AM ZEM] = Guidance_1(R1, SIG1, GAM1, SIGR1)
global GAMD VM1
    
%.. Guidance Law
    % Parameters
    N = 3;
    Tgo = R1/VM1;
    R_dot = -VM1*cos(GAMD-SIG1);
   
    %lateral position
    y = R1*(GAMD-SIG1);
    
    %lateral velocity
    v = VM1*(GAM1-GAMD);
    
   % Guidance Command
   
   % lateral Accelration 
   AM(:) = (-N/Tgo^2)*(y+v*Tgo);
   
   %  Zero miss error
    ZEM = (y+v*Tgo);
end