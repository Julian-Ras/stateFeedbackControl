%% System Ax + Bu
A     = [0 1 0 0; 0 0 1 0; 0 0 0 1; 0 0 0 0];
B     = [0; 0; 0; 1]; 
B1     = [0; 0; 0; 9.8];
p     = [-1,-2,-3,-4];
K     = place(A,B,p); %Controller gains

dt    = 0.001;  %Time step
t     = 50;     %Sim time
s     = 0:dt:t; %Time

x = zeros(size(A,1),length(s)); %State vector
x(:,1) = [0; 0; 0; 0];    %Initial condotions
e = zeros(size(A,1),length(s)); %Error vector
r = zeros(size(A,1),length(s)); %Reference vector
u = zeros(1,length(s));
%% Solver and control
for i = 1 : length(s)
    r(:,i) = [4 + sin(0.5*(i-1)*dt); 0.5*cos(0.5*(i-1)*dt); -0.25*sin(0.5*(i-1)*dt); -0.125*cos(0.5*(i-1)*dt)];%Reference
    xpppp  = 0.0625*sin(0.5*(i-1)*dt);
    e(:,i) = x(:,i) - r(:,i); %Error
    
    u(i) = xpppp + (-K(1)*e(1,i)-K(2)*e(2,i)-K(3)*e(3,i)-K(4)*e(4,i))/9.8;   %State FeedBack
    % u =  -K(1)*e(1,i)-K(2)*e(2,i)-K(3)*e(3,i)-K(4)*e(4,i);   %State FeedBack

    x(:,i+1) = x(:,i) + dt*(A*x(:,i) + B1*u(i)- [0;0;0;xpppp]); %Forward Euler
end

%% Plots
%Position plot
figure(1)
grid on; hold on;
plot(s,x(1,1:end-1), 'LineWidth', 2)
plot(s,r(1,:), 'LineWidth', 2);
xlabel({'Time(s)'}, 'interpreter', 'latex', 'fontsize', 14)
ylabel({'Position(m)'}, 'interpreter', 'latex', 'fontsize', 14)
legend({'$x_{1}(t)$','$x_{1ref}(t)$'}, 'interpreter', 'latex', 'location', 'best', 'fontsize', 14);


%Errors (e1->x, e2->xp, e3->xpp, e4->xppp)
figure(2)
grid on; hold on;
plot(s,e, 'LineWidth', 2);
xlabel({'Time'}, 'interpreter', 'latex', 'fontsize', 14)
ylabel({'Errors'}, 'interpreter', 'latex', 'fontsize', 14)
legend({'$e_{1}(t)$','$e_{2}(t)$','$e_{3}(t)$','$e_{4}(t)$'}, 'interpreter', 'latex', 'location', 'best', 'fontsize', 14);

%Control input plot
figure(3)
grid on; hold on;
plot(s,u, 'LineWidth', 2)
xlabel({'Time'}, 'interpreter', 'latex', 'fontsize', 14)
ylabel({'Control input'}, 'interpreter', 'latex', 'fontsize', 14)


