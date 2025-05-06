%% Get experiment data

get_position_data;
get_velocity_data;
get_acceleration_data;



%% Plot joint position data
figure
subplot(3,1,1);
hold on
plot(joint_position(:,1)-joint_position(1,1), joint_position(:,2), 'LineWidth',1.5);
plot(joint_position(:,1)-joint_position(1,1), joint_position(:,3), 'LineWidth',1.5);
ylabel('angular position $[rad]$','interpreter','latex', 'fontSize',14);
xlabel('time [s]','interpreter','latex', 'fontSize',14);
legend('$\theta_1$','$\theta_2$','interpreter','latex', 'fontSize',14);
xlim([0 joint_position(end,1)-joint_position(1,1)])
grid on
grid minor


%% Plot joint velocity data
subplot(3,1,2);
hold on
plot(joint_velocity(:,1)-joint_velocity(1,1), joint_velocity(:,2), 'LineWidth',1.5);
plot(joint_velocity(:,1)-joint_velocity(1,1), joint_velocity(:,3), 'LineWidth',1.5);
ylabel('angular velocity $[rad/s]$','interpreter','latex', 'fontSize',14);
xlabel('time [s]','interpreter','latex', 'fontSize',14);
legend('$\dot{\theta}_1$','$\dot{\theta}_2$','interpreter','latex', 'fontSize',14);
xlim([0 joint_velocity(end,1)-joint_velocity(1,1)])
grid on
grid minor



%% Plot joint acceleration data
subplot(3,1,3);
hold on
plot(joint_acceleration(:,1)-joint_acceleration(1,1), joint_acceleration(:,2), 'LineWidth',1.5);
plot(joint_acceleration(:,1)-joint_acceleration(1,1), joint_acceleration(:,3), 'LineWidth',1.5);
ylabel('angular acceleration $[rad/s^2]$','interpreter','latex', 'fontSize',14);
xlabel('time [s]','interpreter','latex', 'fontSize',14);
legend('$\ddot{\theta_1}$','$\ddot{\theta_2}$','interpreter','latex', 'fontSize',14);
xlim([0 joint_acceleration(end,1)-joint_acceleration(1,1)])
grid on
grid minor

