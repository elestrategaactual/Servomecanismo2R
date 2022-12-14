%{
    Realiza la Cinemática inversa del mecanismo 2R.

    @param  l1 será la longitud del primer eslabón,
            l2 la longitud del segundo; mientras que
            x,y corresponden a las coordenadas
            cartesianas del extremo distal del segundo
            eslabón del mecanismo.
    @return Devuelve las posiciones angulares de los
            eslabones, q1 corresponde a la posición
            angular del primer eslabón en radianes;
            mientras que q2 será la posición angular
            del segundo eslabón respecto a q1. Buscando
            la trayectoria más óptima de desplazamiento.
%}

function[q1,q2]=Cinversa(l1,l2,x,y,q10,q20)
cnst=(x^2+y^2-l1^2-l2^2)/(2*l1*l2);

q2a=atan2(+sqrt(1-cnst^2),cnst);
q2b=atan2(-sqrt(1-cnst^2),cnst);

q1a=atan2(y,x)-atan2(l2*sin(q2a),l1+l2*cos(q2a));
q1b=atan2(y,x)-atan2(l2*sin(q2b),l1+l2*cos(q2b));

if abs(q10-q1a+q20-q2a)>abs(q10-q1b+q20-q2b)    %seleccionar la ruta mas corta
    q1=q1b;
    q2=q2b;
else
    q1=q1a;
    q2=q2a;
end

