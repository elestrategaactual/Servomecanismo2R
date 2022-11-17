%{
    Traslada la posición dentro del trébol en
    coordenadas polares, a coordenadas cartesianas
    con origen en la ubicación del eslabón fijo.

    @param  lcx, y lcy son respectivamente las distancias
            horizontal y vertical entre el origen donde
            se ubica el eslabón fijo y el centro del trébol.
            Los parámetros theta y r, son las coordenadas 
            polares de cualquier punto en la trayectoria del trébol.
            theta debe estar en radianes, y r en las mismas unidades
            de lcx y lcy.
    @return Devuelve las posiciones cartesianas del 
            punto en la trayectoria del trébol en las
            unidades en que se ingresan los parámetros, 
            comenzando con la coordenada horizontal [x] 
            siguiendo con la coordenada vertical [y].
%}

function[x,y]=Totrigen(lcx,lcy,theta,r)
x=lcx+r.*cos(theta);
y=lcy+r.*sin(theta);