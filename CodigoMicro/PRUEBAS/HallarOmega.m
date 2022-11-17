function [Omega] = HallarOmega(Angle)
    deltaT = 0.001;
    Omega = zeros(length(Angle),1);%diff(Angle);
     for i=2:length(Angle)-1
         Omega(i) = (Angle(i)-Angle(i-1))/deltaT;%diff(Angle);
     end
end