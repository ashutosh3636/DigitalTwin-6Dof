ur = loadrobot('universalUR5');
ur.DataFormat = 'column';
ur.Gravity=[0 0 -9.81;];
a = show(ur);
plot3(waypoint(1,:),waypoint(2,:),waypoint(3,:),'--r','LineWidth',2,'Parent',a)
hold off
close
%open_system('kinematic1.slx')
%sim('kinematic1.slx')
figure('Visible','on');
tformIndex = 1;
q=configs.Data;
for i = 1:10:numel(q)/6
    currConfig1 =q(:,1,i);
    show(ur,currConfig1);
    drawnow

    xyz(tformIndex,:) = tform2trvec(getTransform(ur,currConfig1,"tool0"));
    tformIndex = tformIndex + 1;
end
figure('Visible','on')
show(ur,configs.Data(:,1,end));

hold on
plot3(xyz(:,1),xyz(:,2),xyz(:,3),'-k','LineWidth',3);
pos=trajec.Data;
plot3(pos(1,:),pos(2,:),pos(3,:),'--r','LineWidth',3);
grid on;

%tim= velocity.time;
%vel=((velocity.Data));
% v1=vel(:,1);
% plot(tim,vel);
% grid on;