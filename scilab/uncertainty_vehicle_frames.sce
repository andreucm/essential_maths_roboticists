//______________________________________________________________________________

//A lidar sensor is mounted onboard a mobile platform, at position (1,0.5) with respect to the platform reference center and oriented 45º with respect to the forward axis of the platform. The lidar is "seeing" a point called "q", at 5m with and angle of 30º from its forward axis. 
//The lidar noise is modelled as Gaussian with standard deviations of 2cm for range and 0.025º for azimuth. 

//Express the covariance matrix of the point in cartesian coordinates referenced at lidar frame and referenced at platform frame in the two following situations: 

//CASE 1: The mounting point of the sensor is known with complete precision (null uncertainty).

//CASE 2: The mounting point of the sensor is known with some degree of uncertainty, given by the standard deviation of 1mm for mounting position and 0.5º for mounting orientation

//______________________________________________________________________________

//clear
clear;

//include files (where draw_ellispes_from_cov() function is defined)
exec("/home/andreu/reporting/notes/essential_maths_roboticists/scilab/ellipsesAxis.sci");

mx = 1;
my = 0.5; 
b = %pi/4;
a = 30*%pi/180;
r = 5; 
Cra = [0.02^2 0; 0 (0.025*%pi/180)^2]

// Jacobian of polar to cartesian (linealization)
Jra = [cos(a) -r*sin(a); sin(a) r*cos(a)];

//covariance of point q wrt Lidar frame through uncertainty propagation
CqS =Jra*Cra*Jra'; 

//transform: Lidar frame wrt Base frame
T_LB = [cos(b) -sin(b) mx; sin(b) cos(b) my; 0 0 1];

//************** CASE 1: Sensor mounting point is not uncertain

// covariance of point q wrt Base frame through uncertainty propagation
CqB_1 = T_LB(1:2,1:2)*CqS*T_LB(1:2,1:2)'

//************** CASE : Sensor mounting point is uncertain

//Jacobian of transform when mounting point is uncertain
Jq_B = [cos(b) -sin(b) 1 0 -r*cos(a)*sin(b)-r*sin(a)*cos(b); sin(b) cos(b) 0 1 r*cos(a)*cos(b)-r*sin(a)*sin(b)];

//covariance of point q wrt Base frame through uncertainty propagation
Cmb = [(0.001)^2 0 0; 0 (0.001)^2 0; 0 0 (0.1*%pi/180)^2];
CqB_2 = Jq_B*[CqS zeros(2,3);zeros(3,2) Cmb]*Jq_B';

disp(CqB_1);
disp(CqB_2);


//****************** Drawing
figure('BackgroundColor',[1 1 1]);
drawaxis();
ph = gca(); // handle
ph.isoview = 'on';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
ph.auto_clear = 'off';

//Cq_L
draw_ellispes_from_cov([0 0], CqS,ph);
e=gce(); // get the current entity (the last created, the ellipses)
set(e,"foreground",1);
set(e,"thickness",3);
ph.auto_clear = 'off';

//Cq_B_1
draw_ellispes_from_cov([0 0], CqB_1,ph);
e=gce(); // get the current entity (the last created, the ellipses)
set(e,"foreground",2);
set(e,"thickness",3);

//Cq_B_2
draw_ellispes_from_cov([0 0], CqB_2,ph);
e=gce(); // get the current entity (the last created, the ellipses)
set(e,"foreground",2);
set(e,"thickness",3);

