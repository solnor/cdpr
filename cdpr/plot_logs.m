close all;

i = 1;
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\q.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\qd.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\eint.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\wint.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\t.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\t0.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\f.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\vel.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\fvel.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\wd.txt');
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
fid = fopen('C:\Users\solvno\source\repos\cdpr\cdpr\logs\wa.txt');
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

% subplot(3,1,1)
% plot(t, q(1,:))
% title("Plot of x", "Interpreter", "latex")
% legend("x", "$x_d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,2)
% plot(t, q(2,:))
% title("Plot of y", "Interpreter", "latex")
% legend("y", "$y_d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Position (m)", "Interpreter","latex")
% 
% subplot(3,1,3)
% plot(t, rad2deg(q(3,:)))
% title("Plot of $\phi$", "Interpreter", "latex")
% legend("$\phi$", "$\phi _d$","Interpreter", "latex")
% xlabel("Time (s)", "Interpreter","latex")
% ylabel("Angle (deg)", "Interpreter","latex")






figure(15)
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


figure(18)
subplot(3,1,1)
plot(t , wd(1,:), t ,wa(1,:));
title("Plot of x wrench", "Interpreter", "latex")
legend("$x_d$ - desired", "$x_a$ - actual","Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Force [N]", "Interpreter","latex")

subplot(3,1,2)
plot(t , wd(2,:), t ,wa(2,:));
title("Plot of y wrench", "Interpreter", "latex")
legend("$y_d$ - desired", "$y_a$ - actual","Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Force [N]", "Interpreter","latex")

subplot(3,1,3)
plot(t , wd(3,:), t, wa(3,:) );
title("Plot of $\phi$ wrench", "Interpreter", "latex")
legend("$\phi_d$ - desired", "$\phi_a$ - actual","Interpreter", "latex")
xlabel("Time [s]", "Interpreter","latex")
ylabel("Torque [Nm]", "Interpreter","latex")

figure(16)
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

figure(17)
plot(q(1,:), q(2,:), qd(1,:), qd(2,:))
legend("Pose", "Desired Pose")
xlim([-0.7, 0.7])
ylim([-0.5, 0.5])
grid on;

figure(23)
plot(t,eint);
legend("Integrated error for $x$", "Integrated error for $y$", "Integrated error for $\phi$","Interpreter", "latex")
figure(24)
plot(t,wint);
legend("Integral contrib. for $x$", "Integral contrib. for $y$", "Integral contrib. for $\phi$","Interpreter", "latex")

figure(11)

subplot(2,2,3)
plot(t, vel(1,:)); hold on;
plot(t, fvel(1,:)); hold off;
title("ODrive0", "Interpreter","latex")
xlabel("Time","Interpreter","latex")
ylabel("Velocity", "Interpreter","latex")
legend('Unfiltered','Filtered')

subplot(2,2,1)
plot(t, vel(2,:)); hold on;
plot(t, fvel(2,:)); hold off;
title("ODrive1", "Interpreter","latex")
xlabel("Time","Interpreter","latex")
ylabel("Velocity", "Interpreter","latex")
legend('Unfiltered','Filtered')

subplot(2,2,2)
plot(t, vel(3,:)); hold on;
plot(t, fvel(3,:)); hold off;
title("ODrive2", "Interpreter","latex")
xlabel("Time","Interpreter","latex")
ylabel("Velocity", "Interpreter","latex")
legend('Unfiltered','Filtered')

subplot(2,2,4)
plot(t, vel(4,:)); hold on;
plot(t, fvel(4,:)); hold off;
title("ODrive3", "Interpreter","latex")
xlabel("Time","Interpreter","latex")
ylabel("Velocity", "Interpreter","latex")
legend('Unfiltered','Filtered')
