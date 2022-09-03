







function[q1,q2]=Cinversa(l1,l2,x,y,q10,q20)
cnst=(x^2+y^2-l1^2-l2^2)/(2*l1*l2);

q2a=atan2(+sqrt(1-cnst^2),cnst);
q2b=atan2(-sqrt(1-cnst^2),cnst);

q1a=atan2(y,x)-atan2(l2*sin(q2a),l1+l2*cos(q2a));
q1b=atan2(y,x)-atan2(l2*sin(q2b),l1+l2*cos(q2b));

if abs(q10-q1a+q20-q2a)>abs(q10-q1b+q20-q2b)%seleccionar la ruta mas corta
    q1=q1b;
    q2=q2b;
else
    q1=q1a;
    q2=q2a;
end

