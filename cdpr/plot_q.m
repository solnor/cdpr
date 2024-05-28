close all

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
%%
save(['C:\Users\eliasoa\OneDrive - NTNU\Documents - Modellering, regulatordesign og ' ...
    'simulering av kabeldrevet robot\General\TTK4900 Masteroppgave\Rapport\Tester og fremstillig av plots\' ...
    'direct_kinematics_test\newMP\pt12'])

%% Plot q
figure
subplot(3,1,1)
plot(t , q(1,:));
title("Plot of x", "Interpreter", "latex")
legend("x", "$x_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,2)
plot(t , q(2,:));
title("Plot of y", "Interpreter", "latex")
legend("y", "$y_d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Position (m)", "Interpreter","latex")

subplot(3,1,3)
plot(t , rad2deg(q(3,:)));
title("Plot of $\phi$", "Interpreter", "latex")
legend("$\phi$", "$\phi _d$","Interpreter", "latex")
xlabel("Time (s)", "Interpreter","latex")
ylabel("Angle (deg)", "Interpreter","latex")

%% Plot xy plot of q and q_d
figure
plot(q(1,:), q(2,:))
legend("Pose")
axis("equal")
grid on;
