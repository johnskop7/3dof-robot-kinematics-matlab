%% Robotics I - 3-DOF Robotic Manipulator Kinematic Simulation
% MATLAB implementation accompanying the analytical kinematic derivation.
% Developed for Robotics I: Analysis, Control, Laboratory at NTUA (2022-2023).
%
% The script generates a periodic task-space trajectory, computes one branch
% of the inverse kinematics, evaluates forward kinematics for visualization,
% obtains joint angular velocities through the inverse linear Jacobian, and
% animates the manipulator motion in 2D and 3D.

%% *** Robot (kinematic) model parameters ***
clear all;
close all;
l0 = 10.0;   %% in cm
l1 = 10.0;
l2 = 20.0;
l3 = 30.0;

%% *** sampling period ***

dt = 0.001; % dt = 1 msec

%% *** DESIRED MOTION PROFILE - TASK SPACE ***
T = 10.0; % 10sec duration of motion
t = 0:dt:T;

%initial/final end-point position --> desired task-space trajectory 
h = 55;
zd0 = h;
zd1 = h;
xd0 = 20;
xd1 = -20;
yd0 = 10;
yd1 = -10;

% Desired trajectory : linear movement(x0,y0,z0)-->(x1,y1,z1)-->(x0,y0,z0); Time duration: T;
xd(1) = xd0; 
yd(1) = yd0;
zd(1) = zd0;
xd_dot(1) = 0;
yd_dot(1) = 0;
zd_dot(1) = 0;
lambda_x = 2*(xd1-xd0)/T; %mean linear velocity in the x axis
lambda_y = 2*(yd1-yd0)/T; %mean linear velocity in the y axis
lambda_z = 2*(zd1-zd0)/T; %mean linear velocity in the z axis

%The values of the maximum velocitites are computed in the report of the
%project along with the values of the acceleration in each axis

u_z_max = 0; %maximum velocity in the z axis during the desired trajectory
u_x_max = 12; %maximum velocity in the x axis during the desired trajectory
u_y_max = 6; %maximum velocity in the y axis during the desired trajectory

kmax=T/dt + 1; 

%acceleration of the end-effector during the first half of the trajectory:(x0,y0,z0)-->(x1,y1,z1) 
for k = 1:(kmax/6)
    a_x(k) = 7.2;
    a_y(k) = 3.6;
    a_z(k) = 0;
end

for k = round(kmax/6):2*(kmax/6)
    a_x(k) = 0;
    a_y(k) = 0;
    a_z(k) = 0;
end

for k = 2*round(kmax/6):3*(kmax/6)
    a_x(k) = -7.2;
    a_y(k) = -3.6;
    a_z(k) = 0;
end

%acceleration of the end-effector during the second half of the trajectory:(x0,y0,z0)-->(x1,y1,z1) 
for k = 3*(round(kmax/6)):4*(kmax/6)
    a_x(k) = 7.2;
    a_y(k) = 3.6;
    a_z(k) = 0;
end

for k = 4*round(kmax/6):5*(kmax/6)
    a_x(k) = 0;
    a_y(k) = 0;
    a_z(k) = 0;
end

for k = 5*round(kmax/6):kmax
    a_x(k) = -7.2;
    a_y(k) = -3.6;
    a_z(k) = 0;
end

for k=2:(kmax/2)    
   xd_dot(k) = xd_dot(k-1) + a_x(k)*dt;
   yd_dot(k) = yd_dot(k-1) + a_y(k)*dt;
   zd_dot(k) = zd_dot(k-1) + a_z(k)*dt;
   xd(k) = xd(k-1) + lambda_x*dt;    
   yd(k) = yd(k-1) + lambda_y*dt;
   zd(k) = zd(k-1) + lambda_z*dt;
end  

for k = (round(kmax/2)):kmax 
    xd(k) = xd(k-1) - lambda_x*dt;    
    yd(k) = yd(k-1) - lambda_y*dt;
    zd(k) = zd(k-1) - lambda_z*dt;
    xd_dot(k) = xd_dot(k-1) - a_x(k)*dt;
    yd_dot(k) = yd_dot(k-1) - a_y(k)*dt;
    zd_dot(k) = zd_dot(k-1) - a_z(k)*dt;
end

%% ***** INVERSE KINEMATICS --> DESIRED MOTION - JOINT SPACE ****
%% we use one of the solutions that we found in the theoretical analysis
qd(:,3) = acos((xd(:).^2 + (zd(:) - l0).^2 + yd(:).^2 - l1^2 - l2^2 - l3^2)./(2*l2*l3));
num = cos(qd(:,3))*l3 +l2 + sqrt(l2^2+l3^2+2*l2*l3 - yd(:).^2);
den = sin(qd(:,3))*l3 + yd(:);
qd(:,2) = 2*atan(num./den);
a = atan2(l1 , cos(qd(:,2)+ qd(:,3))*l3 + cos(qd(:,2))*l2);
qd(:,1) = atan2(zd(:) - l0, xd(:)) - a(:);

%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS *****
xd1 = zeros(10001,1);
yd1 = zeros(10001,1);
zd1 = l0*ones(10001,1);
xd2 = -l1*sin(qd(:,1));
yd2 = zeros(10001,1);
zd2 = l0 + l1*cos(qd(:,1));
xd3 = l2*cos(qd(:,1)).*cos(qd(:,2)) - l1*sin(qd(:,1));
yd3 = l2*sin(qd(:,2));
zd3 = l2*sin(qd(:,1)).*cos(qd(:,2)) + l1*cos(qd(:,1)) + l0;
xd4 = l3*cos(qd(:,1)).*cos(qd(:,2)+qd(:,3)) + l2*cos(qd(:,1)).*cos(qd(:,2)) - l1*sin(qd(:,1));
yd4 = l3*sin(qd(:,2)+qd(:,3)) + l2*sin(qd(:,2));
zd4 = l3*sin(qd(:,1)).*cos(qd(:,2)+qd(:,3)) + l2*sin(qd(:,1)).*cos(qd(:,2)) + l1*cos(qd(:,1)) + l0;

%% ***** WE USE THE INVERSE JACOBIAN TO COMPUTE THE ANGULAR VELOCITY OF THE JOINTS****
c1 = cos(qd(:,1));
s1 = sin(qd(:,1));
c2 = cos(qd(:,2));
s2 = sin(qd(:,2));
s3 = sin(qd(:,3));
c3 = cos(qd(:,3));
c23 = cos(qd(:,2)+qd(:,3));
s23 = sin(qd(:,2)+qd(:,3));

a1 = -(s1(:)./(l3*c23(:) + l2*c2(:)));
a2 = 0 ;
a3 = c1./ (l3*c23(:) + l2*c2(:));

b1 = c23(:).*(l3*c1(:).*c23(:) + l2*c1(:).*c2(:) - l1*s1(:))./(l2*s3(:).*(l3*c23(:) + l2*c2(:)));
b2 = s23(:)./(l2*s3(:));
b3 = (c23(:).*(l3*s1(:).*c23(:) + l2*s1(:).*c2(:) + l1*c1(:)))./(l2*s3(:).*(l3*c23(:) + l2*c2(:)));

k1 = -(l3*c1(:).*c23(:) + l2*c1(:).*c2(:) - l1*s1(:))./(l2*l3*s3(:));
k2 = -(l3*s23(:) + l2*s2(:))./(l2*l3*s3(:));
k3 = -(l3*s1(:).*c23(:) + l2*s1(:).*c2(:) + l1*c1(:))./(l2*l3*s3(:));

q1_dot = a1(:).*xd_dot(:) + a2(:).*yd_dot(:) + a3(:).*zd_dot(:) ;
q2_dot = b1(:).*xd_dot(:) + b2(:).*yd_dot(:) + b3(:).*zd_dot(:) ;
q3_dot = k1(:).*xd_dot(:) + k2(:).*yd_dot(:) + k3(:).*zd_dot(:) ;

%% *** PLOT output data *** 
fig1 = figure;  

subplot(1,3,1); 
plot(t,xd); 
ylabel('xd (cm)'); 
xlabel('time t (sec)');  

subplot(1,3,2); 
plot(t,yd); 
ylabel('yd (cm)'); 
xlabel('time t (sec)');

subplot(1,3,3); 
plot(t,zd); 
ylabel('zd (cm)'); 
xlabel('time t (sec)');
sgtitle("End effector's position")

fig2 = figure;

subplot(1,3,1); 
plot(t , xd_dot); 
ylabel('xd dot (cm/s)'); 
xlabel('time t (sec)');  

subplot(1,3,2); 
plot(t , yd_dot); 
ylim([-8 8])
ylabel('yd dot (cm/s)'); 
xlabel('time t (sec)');  

subplot(1,3,3); 
plot(t , zd_dot); 
ylabel('zd dot (cm/s)'); 
xlabel('time t (sec)'); 
sgtitle("End effector's velocity")

fig3 = figure; 

subplot(1,3,1); 
plot(t,qd(:,1)); 
ylabel('qd1 (rad)'); 
xlabel('time t (sec)');  

subplot(1,3,2); 
plot(t,qd(:,2)); 
ylabel('qd2 (rad)'); 
xlabel('time t (sec)');

subplot(1,3,3); 
plot(t,qd(:,3)); 
ylabel('qd3 (rad)'); 
xlabel('time t (sec)');
sgtitle("Angular movement of the joints ")

fig4 = figure; 

subplot(1,3,1); 
plot(t,q1_dot); 
ylabel('q1 dot (rad/s)'); 
xlabel('time t (sec)');  

subplot(1,3,2); 
plot(t,q2_dot); 
ylabel('q2 dot (rad/s)'); 
xlabel('time t (sec)');    

subplot(1,3,3); 
plot(t,q3_dot); 
ylabel('q3 dot (rad/s)'); 
xlabel('time t (sec)'); 
sgtitle("Angular velocities of the joints")

%% stick diagram --> animate robot motion 
fig5 = figure; 
axis([-40 40 -10 70 -70 70]) %%set xyz plot axes 
axis on 
hold on 
plot3(xd,zd,yd,'rs'); 
dtk=500; %% plot robot position every dtk samples, to animate its motion 
plot3([0],[0],[0],'o'); 
for tk=1:dtk:kmax;   
   pause(0.5);	%% pause motion to view successive robot configurations    
   plot3([0,xd1(tk)],[0,zd1(tk)],[0,yd1(tk)]);					
   plot3([xd1(tk)],[zd1(tk)],[yd1(tk)],'o'); 
   plot3([xd1(tk),xd2(tk)],[zd1(tk),zd2(tk)],[yd1(tk),yd2(tk)]);	
   plot3([xd2(tk)],[zd2(tk)],[yd2(tk)],'o');
   plot3([xd2(tk),xd3(tk)],[zd2(tk),zd3(tk)],[yd2(tk),yd3(tk)]);	
   plot3([xd3(tk)],[zd3(tk)],[yd3(tk)],'o');
   plot3([xd3(tk),xd(tk)],[zd3(tk),zd(tk)],[yd3(tk),yd(tk)]);
   plot3([xd(tk)],[zd(tk)],[yd(tk)],'g+'); 
end

fig6 = figure; 
axis([-30 30 -30 30]) %%set xy plot axes 
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
plot(xd,yd,'rs'); 
dtk=500; %% plot robot position every dtk samples, to animate its motion 
plot([0],[0],'o');   
for tk=1:dtk:kmax;   
    pause(0.1);	%% pause motion to view successive robot configurations    
    plot([0,xd1(tk)],[0,yd1(tk)]);					
    plot([xd1(tk)],[yd1(tk)],'o');    
    plot([xd1(tk),xd2(tk)],[yd1(tk),yd2(tk)]);	
    plot([xd2(tk)],[yd2(tk)],'o'); 
    plot([xd2(tk),xd3(tk)],[yd2(tk),yd3(tk)]);	
    plot([xd3(tk)],[yd3(tk)],'o');
    plot([xd3(tk),xd(tk)],[yd3(tk),yd(tk)]);
    plot([xd(tk)],[yd(tk)],'g+');  
end

fig7 = figure; 
axis([-40 40 -10 70]) %%set xy plot axes 
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('z (cm)'); 
plot(xd,zd,'rs'); 
dtk=500; %% plot robot position every dtk samples, to animate its motion 
plot([0],[0],'o');   
for tk=1:dtk:kmax;   
    pause(0.1);	%% pause motion to view successive robot configurations    
    plot([0,xd1(tk)],[0,zd1(tk)]);					
    plot([xd1(tk)],[zd1(tk)],'o');    
    plot([xd1(tk),xd2(tk)],[zd1(tk),zd2(tk)]);	
    plot([xd2(tk)],[zd2(tk)],'o'); 
    plot([xd2(tk),xd3(tk)],[zd2(tk),zd3(tk)]);	
    plot([xd3(tk)],[zd3(tk)],'o');
    plot([xd3(tk),xd(tk)],[zd3(tk),zd(tk)]);
    plot([xd(tk)],[zd(tk)],'g+');  
end
