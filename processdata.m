X = out.X.Data;
Y = out.Y.Data;
x_dot = out.x_dot.data/3.6;
y_dot = out.y_dot.data/3.6;
Phi = out.phi*pi/180;
Phi_dot = out.phi_dot*pi/180;
X_dot = x_dot.*cos(Phi)-y_dot.*sin(Phi);
[Yref, Phiref,Phi_dot_ref] = syx(X);
figure(1);
plot(X,Y,X,Yref);
e_Y = norm(normalize(Y)-normalize(Yref),2)
e_phi = norm(normalize(Phi)-normalize(Phiref),2)
e_phidot = norm(normalize(Phi_dot)-normalize(Phi_dot_ref),2)
legend('Y','Yref');
figure(2);
plot(X,Phi,X,Phiref);
legend('Phi','Phiref');
figure(3);
plot(X,Phi_dot,X,Phi_dot_ref.*X_dot);
legend('Phidot','Phi dot ref');
