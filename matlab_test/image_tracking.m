data = load('test_2.txt');
n = length(data);
% n = 100;

u_hats = zeros(1,n);
v_hats = zeros(1,n);

u_hat = data(1,1);
v_hat = data(1,2);
du_hat = 0;
dv_hat = 0;

u_hats(1) = u_hat;
v_hats(1) = v_hat;

tolerance = 0;
tolerances = zeros(1,n);
dt = 1/60;

Rx = @(t) [1 0 0 ; 0 cos(t) -sin(t) ; 0 sin(t) cos(t)];
Ry = @(t) [cos(t) 0 sin(t) ; 0 1 0 ; -sin(t) 0 cos(t)];
Rz = @(t) [cos(t) -sin(t) 0 ; sin(t) cos(t) 0 ; 0 0 1];

for i=2:n
    u = data(i,1);
    v = data(i,2);
    ez = data(i,3);
    ey = data(i,4);
    ex = data(i,5);
    h = data(i,6);
    R = Rz(ez)*Ry(ey)*Rx(ex);
    
    u_pre = u_hat + du_hat*dt;
    v_pre = v_hat + dv_hat*dt;
    
    d = m_distance(u_pre,v_pre, u,v, R,h);
    dtol = 0.01;
    if d < dtol && tolerance < 1
        tolerance = tolerance + dt/0.5;
    elseif d > dtol && tolerance > 0
        tolerance = tolerance - dt/0.2;
    end
    
    k = 0.35;
%     if tolerance > 0.3 && d > dtol
%         k = 0; % ignore the measurement
%     end
    
    u_hat = u_pre + k*(u - u_pre);
    v_hat = v_pre + k*(v - v_pre);
    
    du_pre = (u_hat - u_hats(i-1))/dt;
    dv_pre = (v_hat - v_hats(i-1))/dt;
    kv = 0.9;
    du_hat = 0.8*(kv*du_hat + (1-kv)*du_pre);
    dv_hat = 0.8*(kv*dv_hat + (1-kv)*dv_pre);
    
    u_hats(i) = u_hat;
    v_hats(i) = v_hat;
    tolerances(i) = tolerance;
end

us = data(1:n,1);
vs = data(1:n,2);

figure(1); clf(1);
hold on; grid on; box on;
plot(1:n, vs);
plot(1:n, v_hats);
% subplot(2,1,2); hold on; grid on; box on;
% plot(1:n, vs);
% plot(1:n, v_hats);
% plot(1:n, data(1:n,2));