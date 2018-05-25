% Problem 4 of assignment 6
% coded by heesoo
% Computation of mechanism using Newton raphson method

clear all
clc
close all

% given initial coordinates 
r = 4;
phi = pi()/4;

q0 = [ r, phi ];

% solution value
cnt = 0;
maxiter = 100;
epsilon = 0.05;
tini = 0;
dt = 0.1;
tend = 50;

% initial evaluation of constraint
C = [q0(1)*cos(q0(2)) - 3 ; 
    q0(1)*sin(q0(2)) - 4 ];

% initial evaluation of jacobian
Cq = [ cos(q0(2)), -q0(1)*sin(q0(2)) ;
       sin(q0(2)), q0(1)*cos(q0(2)) ];
q(:,1) = q0;
tsim(1) = tini;


for t = tini:dt:tend;
    cnt = cnt +1; %Column counter to form the result matrix.
%%%%% 3)Iterative calculations
     nloop=0;
     nconverg=0;
     while nloop<maxiter
     % Newton Difference
deltaq = -Cq^(-1)*C;
% Coordinate update
q(:,cnt)=q(:,cnt)+deltaq;

        nloop=nloop+1;
        nconverg=nconverg+1;
          % Converging criteria (error < epsilon)
          maxdx=max(abs(deltaq));
          %%maxdx=max(abs(error));
          if maxdx<epsilon
             nloop=maxiter;
          end
          tmp=q(:,cnt);
          C = [tmp(1)*cos(tmp(2)) - 3;
               tmp(1)*sin(tmp(2)) - 4];
          % % jacobian update
          Cq = [cos(tmp(2)), -tmp(1)*sin(tmp(2));
                sin(tmp(2)), tmp(1)*cos(tmp(2))];
          tsim(cnt+1)=t;
     end
% Estimation of positions for next step
q(:,cnt+1)=q(:,cnt)
%%)Presentation and store of the result values
q;
nconverg;
%%%% Determination
end