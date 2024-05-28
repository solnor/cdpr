

%% Extract data from .txt file
i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\q.txt');
q = [0;0;0];
while 1
    q_tmp = [str2double(fgetl(fid));
             str2double(fgetl(fid));
             str2double(fgetl(fid))];
    if isnan(q_tmp)
        break;
    end
    q(:,i) = q_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\qdot.txt');
qdot = [0;0;0];
while 1
    qdot_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(qdot_tmp)
        break;
    end
    qdot(:,i) = qdot_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\fqdot.txt');
fqdot = [0;0;0];
while 1
    fqdot_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(fqdot_tmp)
        break;
    end
    fqdot(:,i) = fqdot_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\qddot.txt');
qddot = [0;0;0];
while 1
    qddot_tmp = [str2double(fgetl(fid));
                 str2double(fgetl(fid));
                 str2double(fgetl(fid))];
    if isnan(qddot_tmp)
        break;
    end
    qddot(:,i) = qddot_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\qd.txt');
qd = [0;0;0];
while 1
    qd_tmp = [str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid))];
    if isnan(qd_tmp)
        break;
    end
    qd(:,i) = qd_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\eint.txt');
eint = [0;0;0];
while 1
    eint_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(eint_tmp)
        break;
    end
    eint(:,i) = eint_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\wint.txt');
wint = [0;0;0];
while 1
    wint_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(wint_tmp)
        break;
    end
    wint(:,i) = wint_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\pw.txt');
pw = [0;0;0];
while 1
    pw_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(pw_tmp)
        break;
    end
    pw(:,i) = pw_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\dw.txt');
dw = [0;0;0];
while 1
    dw_tmp = [str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid))];
    if isnan(dw_tmp)
        break;
    end
    dw(:,i) = dw_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\t.txt');
t = 0;
while 1
    t_tmp = str2double(fgetl(fid));
    if isnan(t_tmp)
        break;
    end
    t(i) = t_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\t0.txt');
t0 = [0;0;0;0];
while 1
    t0_tmp = [str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid))];
    if isnan(t0_tmp)
        break;
    end
    t0(:,i) = t0_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\f.txt');
f = [0;0;0;0];
while 1
    f_tmp = [str2double(fgetl(fid));
             str2double(fgetl(fid));
             str2double(fgetl(fid));
             str2double(fgetl(fid))];
    if isnan(f_tmp)
        break;
    end
    f(:,i) = f_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\l.txt');
l = [0;0;0;0];
while 1
    l_tmp = [str2double(fgetl(fid));
             str2double(fgetl(fid));
             str2double(fgetl(fid));
             str2double(fgetl(fid))];
    if isnan(l_tmp)
        break;
    end
    l(:,i) = l_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\ldot.txt');
ldot = [0;0;0;0];
while 1
    ldot_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(ldot_tmp)
        break;
    end
    ldot(:,i) = ldot_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\vel.txt');
vel = [0;0;0;0];
while 1
    vel_tmp = [str2double(fgetl(fid));
               str2double(fgetl(fid));
               str2double(fgetl(fid));
               str2double(fgetl(fid))];
    if isnan(vel_tmp)
        break;
    end
    vel(:,i) = vel_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\fvel.txt');
fvel = [0;0;0;0];
while 1
    fvel_tmp = [str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid));
                str2double(fgetl(fid))];
    if isnan(fvel_tmp)
        break;
    end
    fvel(:,i) = fvel_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\fvel_m_rad.txt');
fvel_m_rad = [0;0;0;0];
while 1
    fvel_m_rad_tmp = [str2double(fgetl(fid));
                      str2double(fgetl(fid));
                      str2double(fgetl(fid));
                      str2double(fgetl(fid))];
    if isnan(fvel_m_rad_tmp)
        break;
    end
    fvel_m_rad(:,i) = fvel_m_rad_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\vel_m_rad.txt');
vel_m_rad = [0;0;0;0];
while 1
    vel_m_rad_tmp = [str2double(fgetl(fid));
                      str2double(fgetl(fid));
                      str2double(fgetl(fid));
                      str2double(fgetl(fid))];
    if isnan(vel_m_rad_tmp)
        break;
    end
    vel_m_rad(:,i) = vel_m_rad_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\wd.txt');
wd = [0;0;0];
while 1
    wd_tmp = [str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid))];
    if isnan(wd_tmp)
        break;
    end
    wd(:,i) = wd_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\wa.txt');
wa = [0;0;0];
while 1
    wa_tmp = [str2double(fgetl(fid));
              str2double(fgetl(fid));
              str2double(fgetl(fid))];
    if isnan(wa_tmp)
        break;
    end
    wa(:,i) = wa_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\t_loop.txt');
t_tmp = 0;
while 1
    t_tmp = str2double(fgetl(fid));
    if isnan(t_tmp)
        break;
    end
    t_loop(i) = t_tmp;
    i = i + 1;
end

i = 1;
fid = fopen('C:\Users\eliasoa\source\repos\cdpr2\cdpr\logs\real_t_loop.txt');
t_tmp = 0;
while 1
    t_tmp = str2double(fgetl(fid));
    if isnan(t_tmp)
        break;
    end
    real_t_loop(i) = t_tmp;
    i = i + 1;
end
hold on
plot(real_t_loop)
plot(t_loop)
hold off
%% Save data
% Base file path and name
base_path = ['C:\Users\eliasoa\OneDrive - NTNU\Documents - ' ...
    'Modellering, regulatordesign og simulering av kabeldrevet robot\General\TTK4900 ' ...
    'Masteroppgave\Rapport\Tester og fremstillig av plots\path_following\newMP\test'];
file_extension = '.mat';
file_number = 1;  % Starting file number

% Generate the initial file name
file_name = [base_path, num2str(file_number), file_extension];

% Check if the file already exists, and increment the number if it does
while exist(file_name, 'file')
    file_number = file_number + 1;
    file_name = [base_path, num2str(file_number), file_extension];
end

% Save the data to the new file name
file_number
save(file_name);
%% Plot q and q_d
figure
subplot(3,1,1)
plot(t , q(1,:), t ,qd(1,:));
title("Plot of x", "Interpreter", "latex")
legend("x", "$x_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,2)
plot(t , q(2,:), t ,qd(2,:));
title("Plot of y", "Interpreter", "latex")
legend("y", "$y_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,3)
plot(t , rad2deg(q(3,:)), t, rad2deg(qd(3,:)) );
title("Plot of $\phi$", "Interpreter", "latex")
legend("$\phi$", "$\phi _d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Angle (deg)", "Interpreter","latex")

%% Plot qdot
figure
subplot(3,1,1)
plot(t , fqdot(1,:), t , qddot(1,:));
title("Plot of xdot", "Interpreter", "latex")
legend("$\dot{x}$", "$\dot{x}_d$", "Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Position [m]", "Interpreter","latex")

subplot(3,1,2)
plot(t , fqdot(2,:), t , qddot(2,:));
title("Plot of y", "Interpreter", "latex")
legend("$\dot{y}$", "$\dot{y}_d$", "Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Position [m]", "Interpreter","latex")

subplot(3,1,3)
plot(t , fqdot(3,:), t , qddot(3,:));
title("Plot of $\dot{\phi}$", "Interpreter", "latex")
legend("$\dot{\phi}$", "$\dot{\phi}_d$", "Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Angular velocity [rad/s]", "Interpreter","latex")

%% Plot of error
figure
subplot(3,1,1)
plot(t , qd(1,:)- q(1,:));
title("Plot of error in x", "Interpreter", "latex")
legend("x", "$x_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,2)
plot(t , qd(2,:) - q(2,:));
title("Plot of error in y", "Interpreter", "latex")
legend("y", "$y_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,3)
plot(t , rad2deg(qd(3,:) - q(3,:)));
title("Plot of error in $\phi$", "Interpreter", "latex")
legend("$\phi$", "$\phi _d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Angle (deg)", "Interpreter","latex")

%% Plot xy plot of q and q_d
figure
plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
legend("Pose", "Desired Pose")
axis("equal")
grid on;
%% Plot xy plot of qdot and qddot
% figure
% plot(qdot(1,:), qdot(2,:), qddot(1,:), qddot(2,:))
% legend("Twist", "Desired Twist")
% % xlim([-0.5, 0.5])
% % ylim([-0.5, 0.5])
% axis("equal")
% grid on;
%% Plot of cable forces, impulses and motor velocities
figure
subplot(3,1,1)
hold on
plot(t,f);
legend('$f_1$','$f_2$','$f_3$','$f_4$',"Interpreter", "latex")
subplot(3,1,2)
plot(t,t0)
legend('$f_1$','$f_2$','$f_3$','$f_4$',"Interpreter", "latex")
subplot(3,1,3)
plot(t,vel)
legend('$v1$','$v2$','$v3$','$v4$',"Interpreter", "latex")

%% Plot of PID parts
figure
subplot(3,1,1)
plot(t,pw(1,:))
title("Plot of P wrench contributions", "Interpreter", "latex")
legend('$w_x$',"Interpreter", "latex")
subplot(3,1,2)
plot(t,pw(2,:))
legend('$w_y$',"Interpreter", "latex")
subplot(3,1,3)
plot(t,pw(3,:))
legend('$w_{\phi}$',"Interpreter", "latex")


figure
subplot(3,1,1)
plot(t,dw(1,:))
title("Plot of D wrench contributions", "Interpreter", "latex")
legend('$w_x$',"Interpreter", "latex")
subplot(3,1,2)
plot(t,dw(2,:))
legend('$w_y$',"Interpreter", "latex")
subplot(3,1,3)
plot(t,dw(3,:))
legend('$w_{\phi}$',"Interpreter", "latex")

figure
subplot(3,1,1)
plot(t,wint(1,:))
title("Plot of I wrench contributions", "Interpreter", "latex")
legend('$w_x$',"Interpreter", "latex")
subplot(3,1,2)
plot(t,wint(2,:))
legend('$w_y$',"Interpreter", "latex")
subplot(3,1,3)
plot(t,wint(3,:))
legend('$w_{\phi}$',"Interpreter", "latex")
% 
%% Plot of wrenches
% figure
% subplot(3,1,1)
% plot(t , wd(1,:), t ,wa(1,:));
% title("Plot of x wrench", "Interpreter", "latex")
% legend("$x_d$ - desired", "$x_a$ - actual","Interpreter", "latex")
% xlabel("Time [s]", "Interpreter","latex")
% ylabel("Force [N]", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(t , wd(2,:), t ,wa(2,:));
% title("Plot of y wrench", "Interpreter", "latex")
% legend("$y_d$ - desired", "$y_a$ - actual","Interpreter", "latex")
% xlabel("Time [s]", "Interpreter","latex")
% ylabel("Force [N]", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(t , wd(3,:), t, wa(3,:) );
% title("Plot of $\phi$ wrench", "Interpreter", "latex")
% legend("$\phi_d$ - desired", "$\phi_a$ - actual","Interpreter", "latex")
% xlabel("Time [s]", "Interpreter","latex")
% ylabel("Torque [Nm]", "Interpreter","latex")

%% Plot of integral error and integral contribution
% figure(9)
% plot(t,eint);
% legend("Integrated error for $x$", "Integrated error for $y$", "Integrated error for $\phi$","Interpreter", "latex")
% 
% figure(10)
% plot(t,wint);
% legend("Integral contrib. for $x$", "Integral contrib. for $y$", "Integral contrib. for $\phi$","Interpreter", "latex")
% 
%% Plot of velocity filtered and unfiltered
% figure(11)
% subplot(2,2,3)
% plot(t, vel(1,:)); hold on;
% plot(t, fvel(1,:)); hold off;
% title("ODrive0", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,1)
% plot(t, vel(2,:)); hold on;
% plot(t, fvel(2,:)); hold off;
% title("ODrive1", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,2)
% plot(t, vel(3,:)); hold on;
% plot(t, fvel(3,:)); hold off;
% title("ODrive2", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,4)
% plot(t, vel(4,:)); hold on;
% plot(t, fvel(4,:)); hold off;
% title("ODrive3", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
%% Plot motor velocities and filtered motor velocities
% figure(12)
% subplot(2,2,3)
% plot(t, vel_m_rad(1,:)); hold on;
% plot(t, fvel_m_rad(1,:)); hold off;
% title("ODrive0", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,1)
% plot(t, vel_m_rad(2,:)); hold on;
% plot(t, fvel_m_rad(2,:)); hold off;
% title("ODrive1", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,2)
% plot(t, vel_m_rad(3,:)); hold on;
% plot(t, fvel_m_rad(3,:)); hold off;
% title("ODrive2", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
% 
% subplot(2,2,4)
% plot(t, vel_m_rad(4,:)); hold on;
% plot(t, fvel_m_rad(4,:)); hold off;
% title("ODrive3", "Interpreter","latex")
% xlabel("Time","Interpreter","latex")
% ylabel("Velocity", "Interpreter","latex")
% legend('Unfiltered','Filtered')
