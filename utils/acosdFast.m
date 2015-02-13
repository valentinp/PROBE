function [ ac ] = acosdFast( x )
%ACOSDFAST Approximation to arccos
a=1.43+0.59*x; 
a=(a+(2+2*x)/a)/2;
b=1.65-1.41*x;
b=(b+(2-2*x)/b)/2;
c=0.88-0.77*x;
c=(c+(2-a)/c)/2;
ac= (8*(c+(2-a)/c)-(b+(2-2*x)/b))/6;
ac = 57.2958*ac; %180/pi
end

