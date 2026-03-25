function [y] = norm_pdf(x,mean,std)
%NORM_PDF Summary of this function goes here
%   Detailed explanation goes here

y=1/(std*sqrt(2*pi))*exp(-(1/2)*((x-mean)/std).^2);


end

