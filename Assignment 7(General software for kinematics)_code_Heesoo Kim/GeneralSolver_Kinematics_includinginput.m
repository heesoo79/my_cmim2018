clear all
close all

% coded by Heesoo Kim for Course assignment7, 2018 May
% General software to perform the general mechanism's kinematic analysis
% Revolution joint is only used

%% Description of Mechanism (ex, four bar linkage case)
% Input joint information
if 1
joint1.type = 'revolute_groundnbody';
joint1.first_body_id = 0; % 0 : ground body
joint1.second_body_id = 1;
joint1.position = [0; 0; 0]; % initial position of joint center
end

if 0
joint2.type = 'free';
joint2.first_body_id = 1; % 0 : ground body
joint2.second_body_id = 0;
joint2.position = [0.1; 0.4; 0];
end

if 1
joint2.type = 'revolute_bodynbody';
joint2.first_body_id = 1;
joint2.second_body_id = 2;
joint2.position = [0.1; 0.4; 0];
end

if 1
joint3.type = 'revolute_bodynbody';
joint3.first_body_id = 2;
joint3.second_body_id = 3;
joint3.position = [0.9; 0.6; 0];
end

if 1
joint4.type = 'revolute_bodynground';
joint4.first_body_id = 3;
joint4.second_body_id = 0;
joint4.position = [0.9; 0; 0];
end

% Input driving condition
driving1.njoint = 1; % ID number of driving joint
driving1.nbody = 1; % ID number of driving body
driving1.revspeed = 2*pi(); % Driving speed
Theta10 = atan2( joint2.position(2)-joint1.position(2), joint2.position(1)-joint1.position(1) ); % initial angle of driving body

% model description of joints
model_description.joints = [joint1,joint2,joint3,joint4];
Nj = length(model_description.joints); % number of joints

% model description of bodies
body1 = struct ;
body2 = struct ;
body3 = struct ;
model_description.bodies = [body1,body2,body3];
Nb = length(model_description.bodies); % number of bodies
for i = 1:Nb
    model_description.bodies(i).length = sqrt((model_description.joints(i+1).position(1)-model_description.joints(i).position(1))^2 + (model_description.joints(i+1).position(2)-model_description.joints(i).position(2))^2);
end

% model discription of drivers
model_description.drivings = [driving1];
Nq = 3*length(model_description.bodies); % number of general coordinates

%%
% Calculation of initial general coordinates
q0 = zeros(Nq,1); 
for i = 1:Nq/3
    q0(3*(i-1)+1) = model_description.joints(i).position(1) + (model_description.joints(i+1).position(1)-model_description.joints(i).position(1))/2 ;
    q0(3*(i-1)+2) = model_description.joints(i).position(2) + (model_description.joints(i+1).position(2)-model_description.joints(i).position(2))/2 ;
    q0(3*(i-1)+3) = atan2( model_description.joints(i+1).position(2)-model_description.joints(i).position(2), model_description.joints(i+1).position(1)-model_description.joints(i).position(1) );
end
dq0 = zeros(Nq,1);
ddq0 = zeros(Nq,1);
% Symbolic variable
syms ts
x = sym('x', [Nq, 1], 'real');
xp = sym('xp', [Nq, 1], 'real');
xpp = sym('xpp', [Nq, 1], 'real');

% maximum number of iterations
maxiter = 20;
% parameter to establish the convergence criteria
epsilon = 0.0001;
% Solution values
cnt = 0;
tini = 0; % Initial time
dt = 0.001; % Time step
tend = 1; % End time
idx = 0;

% Constraint matrix
for i = 1:length(model_description.joints)
    if strcmp(model_description.joints(i).type,'revolute_groundnbody')
        body_id = model_description.joints(i).second_body_id;
        C(idx+(1:2)) = addRevoluteJoint_GroundnBody ( x(3*body_id-2), x(3*body_id-1), x(3*body_id), model_description.bodies(body_id).length, model_description.joints(body_id).position )
        idx = idx + 2;
    elseif strcmp (model_description.joints(i).type,'revolute_bodynbody')
           first_body_id = model_description.joints(i).first_body_id;
           second_body_id = model_description.joints(i).second_body_id;
           C(idx+(1:2)) = addRevoluteJoint_BodynBody ( x(3*first_body_id-2), x(3*first_body_id-1), x(3*first_body_id), x(3*second_body_id-2), x(3*second_body_id-1), x(3*second_body_id),model_description.bodies(first_body_id).length, model_description.bodies(second_body_id).length )
           idx = idx + 2;
    elseif strcmp(model_description.joints(i).type,'revolute_bodynground')
        body_id = model_description.joints(i).first_body_id;
        C(idx+(1:2)) = addRevoluteJoint_BodynGround ( x(3*body_id-2), x(3*body_id-1), x(3*body_id), model_description.bodies(body_id).length, model_description.joints(body_id+1).position )
        idx = idx + 2;
    else
    end
end

% Driving constraints
for i = 1:length(model_description.drivings)
    drivingbody_idx = model_description.drivings(i).nbody;
    C(idx+1) = x(3*drivingbody_idx) - (model_description.drivings(i).revspeed*ts + Theta10)
    idx = idx + 1;
end

% Jacobian
Cq = jacobian(C, x)

% initial evaluation of constraint
Cs0 = subs(C,x,q0);
Cs0 = subs(Cs0,ts,0)
C0 = double(Cs0)
% 
% initial evaluation of jacobian
Cqs0 = subs(Cq,x,q0);
Cq0 = double(Cqs0);

q(:,1)=q0;
dq(:,1)=dq0;
ddq(:,1)=ddq0;

tic
for t = tini:dt:tend;
    cnt = cnt + 1;
    % interative calculations
    nloop = 0;
    nconverg = 0;
    while nloop < maxiter      
        if t == tini
            deltaq = -Cq0\C0';
        else
            deltaq = -Cq1\C1';
        end
    % coordinate update
    q(:,cnt) = q(:,cnt)+deltaq;
    
    nloop = nloop+1;
    nconverg = nconverg + 1;
    % Converging criteria (error < epsilon)
    maxdx = max(abs(deltaq));
    
    if maxdx < epsilon
        nloop = maxiter;        
    end
    % Constraints update
    tmp = q(:,cnt);
    Cs1 = subs(C,x,q(:,cnt));
    Cs1 = subs(Cs1,ts,t);
    C1 = double(Cs1);
    % Jacobian update
    Cqs1 = subs(Cq,x,q(:,cnt));
    Cq1 = double(Cqs1);
    
    tsim(cnt+1) = t;
    end
    % Estimation of position for next step
    q(:,cnt+1) = q(:,cnt);
    % Presentation and store of the result values
    q
    nconverg;   
end

% X-position of local coordinate system
figure
plot(tsim,q(1,:))
set(gca,'YLim',[-0.5 1],'Fontsize',12);
title('X-position of local coordinate system')
xlabel('Time [s]')
ylabel('X-Position of local coordinate system [m]')
grid on
hold on
plot(tsim,q(4,:))
hold on
plot(tsim,q(7,:))
legend('Body1,x', 'Body2,x', 'Body3,x');

% Y-position of local coordinate system
figure
plot(tsim,q(2,:))
title('Y-position of local coordinate system')
set(gca,'YLim',[-0.25 0.55],'Fontsize',12);
grid on
xlabel('Time [s]')
ylabel('Y-Position of local coordinate system [m]')
hold on
plot(tsim,q(5,:))
hold on
plot(tsim,q(8,:))
legend('Body1,y', 'Body2,y', 'Body3,y');

toc

