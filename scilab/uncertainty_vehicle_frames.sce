//clear
clear;

//include files (where draw_ellispes_from_cov() function is defined)
exec("/home/andreu/reporting/notes/essential_maths_roboticists/scilab/ellipsesAxis.sci");

//Sensor point q detection in polar coordinates (r,a) (measurement space)
r_q = 8;
a_q = 23*%pi/180; //rad  (20.467)
qS = [r_q*cos(a_q); r_q*sin(a_q);1]; //point q in homogeneous coordinates wrt to the Sensor

//sensor noise in polar coordinates (measurement space) 
sigma_range = 0.01; //meters 
sigma_angle = 0.1*%pi/180; //rad 
Cra_q = [sigma_range^2 0;0 sigma_angle^2]; //covariance matrix in measurement space
J_ra = [cos(a_q) -r_q*sin(a_q); sin(a_q) r_q*cos(a_q); 0 0]; //Jacobian: Linearization from measurement to homogeneous space

//Sensor mounting point with respect to the vehicle base
betaB_S = 35*%pi/180; //orientation angle of the sensor wrt the base
mB_S = [31;12]; //xy coordinates of the sensor wrt the base
RB_S = [cos(betaB_S) -sin(betaB_S); sin(betaB_S) cos(betaB_S)]; //rotation of the sensor wrt the base (R base2sensor)
TB_S = [RB_S mB_S;0 0 1]; //homogeneous transform of the sensor wrt the base (T base2sensor)

//sensor mounting point uncertainty (calibration error, or on-line sensor frame positioning error)
sigma_mx = 0.02; //meters
sigma_my = 0.02; //meters
sigma_beta = 0.2*%pi/180; //rad
C_mbeta = [sigma_mx^2 0 0; 0 sigma_my^2 0; 0 0 sigma_beta^2];
J_mbeta = [1 0 -qS(1)*sin(betaB_S)-qS(2)*cos(betaB_S); 0 1 qS(1)*cos(betaB_S)-qS(2)*sin(betaB_S); 0 0 0];

//------------------ PROPAGATE COVARIANCES --------------
CS_q = J_ra*Cra_q*J_ra'; disp(CS_q);
CB_q = TB_S*CS_q*TB_S' + J_mbeta*C_mbeta*J_mbeta'; disp(CB_q);

//------------------- DRAW ELLIPSES ---------------------
figure('BackgroundColor',[1 1 1]);
drawaxis();
ph = gca(); // handle
ph.isoview = 'on';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
ph.auto_clear = 'off';

//CS_q
draw_ellispes_from_cov([0 0], CS_q(1:2,1:2),ph);
e=gce(); // get the current entity (the last created, the ellipses)
set(e,"foreground",1);
set(e,"thickness",3);
ph.auto_clear = 'off';

//CB_q
draw_ellispes_from_cov([0 0], CB_q(1:2,1:2),ph);
e=gce(); // get the current entity (the last created, the ellipses)
set(e,"foreground",2);
set(e,"thickness",3);
