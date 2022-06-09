
clear;
clc;
time = 10;
fps = 500;
delta_t = 1/fps;
damp_k = 0.2;
g = 32.7;
power = 5.1;
camera_h = 80;
v = 60 * power;
x = [0:1099];
y = zeros(25,1100);
hold off;
distance = zeros(25,1);
camera_angle_res = zeros(25,1);
for i = [1:25]
    angle = i*pi/180;
    y(i,:) = 25*g*log(1-0.2.*x./(v*cos(angle)))+(v*sin(angle)+5*g)*x/(v*cos(angle))+2;
    %plot(y(i+1,:));
    distance(i) = find(y(i,:)<1,1);
    camera_angle_res(i) = 180/pi*atan(distance(i)/camera_h);
end
axis([0 90 0 90]);
grid on;
axis equal
%plot(camera_angle_res);
gun_dist = 2;
trajectory = zeros(25,2);

for gun_scale = 2
    for camera_scale = 2.5
        for camera_dist = 0.5
            for camera_phase = 21.63
                for i = 1:25
                    point_gun = [0,0];
                    point_camera = [point_gun(1)+gun_dist*cos(gun_scale*pi/180)+camera_dist*sin(camera_phase*pi/180),
                                    point_gun(2)+gun_dist*sin(gun_scale*pi/180)+camera_dist*cos(camera_phase*pi/180)];
                    gun_angle = i*gun_scale;
                    camera_angle = (camera_angle_res(i)-camera_angle_res(1))*camera_scale+camera_phase;
                    syms x;
                    eq_gun = x*tan(gun_angle*pi/180);
                    eq_camera = point_camera(2) + cot(camera_angle*pi/180)*(x-point_camera(1));
                    trajectory(i,1) = solve(eq_gun == eq_camera,x);
                    trajectory(i,2) = trajectory(i,1)*tan(gun_angle*pi/180);
                end
                hold on
                plot(point_camera(1),point_camera(2),'G>',0,0,'r>',trajectory(1:25,1),trajectory(1:25,2),"b--o");
                %plot ([trajectory(1,1), projectory(15,1)],[projectory(1,2),projectory(15,2)],"r","LineWidth",2);
                axis([0 4 0 4]);
                grid on
                axis equal
                pause(0.01)
            end
        end
    end
end



