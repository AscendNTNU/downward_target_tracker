function d=m_distance(u1,v1,u2,v2,R,h)
p1 = m_trace(u1,v1,R,h);
p2 = m_trace(u2,v2,R,h);
d = sqrt((p1-p2)'*(p1-p2));
end