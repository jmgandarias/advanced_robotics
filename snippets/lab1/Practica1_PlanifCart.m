% Lab 1: Interpolación cartesiana
clearvars

P1=[1 0 0 0.3740; 0 1 0 0; 0 0 1 0.6300; 0 0 0 1];
P2=[0 0 1 0.3038; 0 1 0 0; -1 0 0 0.0510; 0 0 0 1];
P3=[0 -1 0 0; 0 0 1 0.3020; -1 0 0 0.5580; 0 0 0 1];

tau=1;
T=10;

%% Apartado 1

[p1, q1]=qpinter(P1, P2, 0)
[p2, q2]=qpinter(P1, P2, 1)

%% Apartado 2

% Cargar modelo robot ABB IRB120
[IRB120, IRB120Data] = loadrobot('abbIrb120','DataFormat','row','Gravity',[0 0 -9.81]);
Home=IRB120.homeConfiguration; % Usar posición home como búsqueda inicial para ik
% crear objeto para cinemática inversa
ik_IRB120 = inverseKinematics('RigidBodyTree', IRB120); 
% tolerancias para el cálculo de orientación (los tres primeros son para la
% orientación
weights=[0.25 0.25 0.25 1 1 1];
% Figura para la representación del robot
f1=figure(1)
set(f1,'Name','Manipulador');

% Cálculo de la interpolación a lo largo de todo el tramo
x=[]; y=[]; z=[]; alfa=[]; beta=[]; gamma=[];
for t=-T:0.1:T
    % Llamada a función de interpolación (tramoq)
    [P,Q]=tramoq(P1,P2,P3,tau,T,t);
    x=[x P(1)];
    y=[y P(2)];
    z=[z P(3)];
    
    Tq=q2tr(Q);
    Tq(1:3,4)=[P(1) P(2) P(3)]'; % matriz de transformación homogénea completa
    ZYZ=tr2zyz(Tq);
    
    alfa=[alfa,ZYZ(1)];
    beta=[beta,ZYZ(2)];
    gamma=[gamma,ZYZ(3)];
   
    % Obtener posición y orientación en el espacio articular (cinemática inversa)
    [robot_pose, solnInfo]=ik_IRB120('tool0',Tq, weights, Home);
    % Representar cada posición del robot 
    ax=show(IRB120, robot_pose); axis([-0.5,0.5,-0.5,0.5,0,1]);   
     hold on
     plot3(ax, x, y, z, 'b*')  
     hold off
    drawnow
end

% Representación de variables
t=-T:0.1:T;
f2=figure(2);
set(f2,'Name','Variables cartesianas');
subplot(3,1,1),plot(t,x);title('X');xlabel('t(s)'); ylabel('metros');
subplot(3,1,2),plot(t,y);title('Y');xlabel('t(s)'); ylabel('metros');
subplot(3,1,3),plot(t,z);title('Z');xlabel('t(s)'); ylabel('metros');
    
% Representación de ángulos de Euler
f3=figure(3);
set(f3,'Name','Ángulos de Euler');
subplot(3,1,1),plot(t,alfa);title('alfa');xlabel('t(s)'); ylabel('rad');
subplot(3,1,2),plot(t,beta);title('beta');xlabel('t(s)'); ylabel('rad');
subplot(3,1,3),plot(t,gamma);title('gamma');xlabel('t(s)'); ylabel('rad');



