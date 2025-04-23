% Lab 1: Cartesian trajectory planning
clearvars

P0=[1 0 0 0.3740; 0 1 0 0; 0 0 1 0.6300; 0 0 0 1];
P1=[0 0 1 0.3038; 0 1 0 0; -1 0 0 0.0510; 0 0 0 1];
P2=[0 -1 0 0; 0 0 1 0.3020; -1 0 0 0.5580; 0 0 0 1];

tau=1;
T=10;

%% Exercise 1: Cartesian interpolation

[p1, q1]=qpinter(P0, P1, 0)  %% (1)!
[p2, q2]=qpinter(P0, P1, 1)

%% Exercise 2: Smooth trajectory generation

% Load ABB IRB120 model
[IRB120, IRB120Data] = loadrobot('abbIrb120','DataFormat','row','Gravity',[0 0 -9.81]);
Home=IRB120.homeConfiguration; % Use home position as start setting for inverser kinematics
% Create object for inverse kinematics
ik_IRB120 = inverseKinematics('RigidBodyTree', IRB120); 
% Tolerances
weights=[0.25 0.25 0.25 1 1 1];
% Create figure to represent the manipulator
f1=figure(1)
set(f1,'Name','Manipulador');

% Calculate the interpolation for the whole segment
x=[]; y=[]; z=[]; alfa=[]; beta=[]; gamma=[];
for t=-T:0.1:T
    % Call the function to generate the smoothed cartesian path
    [P,Q]=generate_smooth_path(P0,P1,P2,tau,T,t); %% (2)!
    x=[x P(1)];
    y=[y P(2)];
    z=[z P(3)];
    
    Tq=q2tr(Q);
    Tq(1:3,4)=[P(1) P(2) P(3)]'; % Complete homogeneous transofrmation matrix
    ZYZ=tr2zyz(Tq);
    
    alfa=[alfa,ZYZ(1)];
    beta=[beta,ZYZ(2)];
    gamma=[gamma,ZYZ(3)];
   
    % Get position and orientation in joint space (ik)
    [robot_pose, solnInfo]=ik_IRB120('tool0',Tq, weights, Home);
    % Represent each configuration of the robot
    ax=show(IRB120, robot_pose); axis([-0.5,0.5,-0.5,0.5,0,1]);   
     hold on
     plot3(ax, x, y, z, 'b*')  
     hold off
    drawnow
end

%% Exercise 3: Graphical representation

% Represent cartesian position
t=-T:0.1:T;
f2=figure(2);
set(f2,'Name','Cartesian position');
subplot(3,1,1),plot(t,x);title('X');xlabel('t [s]'); ylabel('position [m]');
subplot(3,1,2),plot(t,y);title('Y');xlabel('t [s]'); ylabel('position [m]');
subplot(3,1,3),plot(t,z);title('Z');xlabel('t [s]'); ylabel('position [m]');
    
% Represent cartesian orientation as Euler angles
f3=figure(3);
set(f3,'Name','Ángulos de Euler');
subplot(3,1,1),plot(t,alfa);title('alfa');xlabel('t [s]'); ylabel('angle [rad]');
subplot(3,1,2),plot(t,beta);title('beta');xlabel('t [s]'); ylabel('angle [rad]');
subplot(3,1,3),plot(t,gamma);title('gamma');xlabel('t [s]'); ylabel('angle [rad]');



