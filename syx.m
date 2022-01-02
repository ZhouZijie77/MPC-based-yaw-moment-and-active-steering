function [Yref, Phiref,Phi_dot_ref]= syx(X)
%OUT 此处显示有关此函数的摘要
%   此处显示详细说明
shape = 2.4;
dx1=25;dx2=21.95;
dy1=4.05;dy2=5.7;
Xs1=27.19;Xs2=56.46;
% Xs1=40;Xs2=56.46;
z1 = shape/dx1*(X-Xs1)-1.2;
z2 = shape/dx2*(X-Xs2)-1.2;
dx1 = 25;
dx2 = 21.95;
dy1 = 4.05;
dy2 = 5.7;
Yref = dy1/2*(1+tanh(z1))-dy2/2*(1+tanh(z2));
Phiref = atan(dy1*(1./cosh(z1)).^2*(1.2/dx1)-dy2*(1./cosh(z2)).^2*(1.2/dx2));
m = dy1.*(1./cosh(z1)).^2*(1.2/dx1)-dy2.*(1./cosh(z2)).^2*(1.2/dx2);
Phi_dot_ref = 1./(1+m.^2).*(-2.4.*shape).*((cosh(z1)).^(-3).*sinh(z1)*dy1/dx1^2-cosh(z2).^(-3).*sinh(z2)*dy2/dx2^2);
end


