%{
    Se obtienen los vectores en coordenadas polares de la
    trayectoria del trébol.

    @param  El primer parámetro rotacion, hace referencia
            a la rotación angular en radianes requerida del trébol,
            Se debe encontrar en el intervalo [-pi/4, pi/4].
            lmin, es la longitud de la arista en la que se debe
            inscribir el trébol. factor, permite agrandar el trébol 
            debe estar en el intervalo [1, 4/3]. es, se trata de 
            un parámetro de la estilización del trébol. hojas, 
            se refiere al número de hojas del trébol. n, es el 
            número de puntos con el que se discretiza la función.
    @return Devuelve las posiciones polares de cada 
            punto en la trayectoria del trébol.
%}

function[r,theta]=trayectoria0(rotacion,lmin, factor,es,hojas,n)
theta=linspace(0,2*pi,n);
rotacion = ((rotacion+(pi/4))/(pi/4));  %Normalizamos la rotación entre 0 y 2
diagonalLmin = sqrt(((lmin/2)^2)+((lmin/2)^2)); %Encontramos la diagonal del cuadrado base
%abase = (diagonalLmin)/(cos(pi/4)+es);    %Encontramos el parámetro a base
thetaMax = 52.5*pi/180;
abase = (lmin/2)/(sin(thetaMax)*(es+cos(4*thetaMax+pi)));
r=(factor*abase)*(es+cos(hojas*theta+rotacion*pi));
factor*abase
end
