function [x,y]=m_trace(u,v,R,h)
f = 494 / 8;
u0 = 649 / 8;
v0 = 335 / 8;
platez = 0.1;

du = u-u0;
dv = -(v-v0);
r = sqrt(du*du + dv*dv);
dir = [0 0 -1]';
if r > 1
    t = r/f;
    s = sin(t);
    c = cos(t);
    dir = [s*du/r s*dv/r -c]';
end

dir = R*dir;

% todo: dir(3) == 0
t = -(h-platez)/dir(3);
x = t*dir(1);
y = t*dir(2);

end