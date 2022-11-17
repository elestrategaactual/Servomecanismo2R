%{
    Realiza la Cinemática directa del mecanismo 2R.

    @param  l1 será la longitud del primer eslabón,
            l2 la longitud del segundo; mientras que
            q1 será el ángulo en radianes del primer
            eslabón; mientras que q2 será el ángulo del
            del segundo eslabón respecto a la posición
            angular del primero en radianes.
    @return Devuelve las posiciones cartesianas del 
            extremo distal del segundo eslabón en las
            unidades en que se ingresan las longitudes
            de los eslabones, comenzando con la 
            coordenada horizontal [x] siguiendo con 
            la coordenada vertical [y].
%}

function[x,y]=Cdirecta(l1,l2,q1,q2)
x=l1*cos(q1)+l2*cos(q1+q2);
y=l1*sin(q1)+l2*sin(q1+q2);

