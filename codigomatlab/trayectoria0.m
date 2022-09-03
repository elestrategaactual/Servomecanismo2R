function[r,theta]=trayectoria0(rotacion, factor,es,hojas,n)
theta=linespace(0,2*pi,n);
r=(factor/es)*(es+sin(hojas*theta+rotacion*pi));
