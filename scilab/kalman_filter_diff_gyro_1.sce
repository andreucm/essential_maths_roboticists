//______________________________________________________________________________

// A Kalman filter to estimate the state of a differential drive robot equipped
// with wheel encoders and one gyro.
// The state is [px py th vx wz]^T  (m,m,rad,m/s,rad/s)
// It can be observed that the uncertainty of the px,py,th components is not
// bounded since there is no direct/indirect correction for them.
//______________________________________________________________________________


// clear
clear;

// input params
time_step = 0.01; // algorithm loop rate [s]
base_length = 1;// base length [m]
wheel_radius = 0.4;// wheel radius [m]
std_dev_wheel_encoder = ((2*%pi)/1024)/time_step;//[rad/s]
std_dev_gyro = 0.01;//[rad/s]
vx_max = 2; //[m/s]
wz_max = %pi; //[rad/s]
ax_max = 5; //[m/s^2]
wz_dot_max = %pi; //[rad/s^2]

// simulation params
n_iter = 2000;//[#]
vx_true_amp = 2; //[m/s]
vx_true_period = 5; //[s]
wz_true_amp = 0.1; //[rad/s]
wz_true_period = 10; //[s]
omega_noise = std_dev_wheel_encoder; //std dev[rad/s]
gyro_noise = std_dev_gyro; //std dev[rad/s]
rand("normal");//set the distribution type to the generator

// initializations
//I-kinematics to compute true rear wheel rotation speed and true steering
x = [0;0;0;0;0];
Cx = zeros(5,5);
Cx(4,4) = 1;
Cx(5,5) = 1;
Cnz = [std_dev_wheel_encoder^2 0 0; 0 std_dev_wheel_encoder^2 0; 0 0 std_dev_gyro^2];
Cnx = zeros(5,5);
Cnx(1,1) = (0.1*vx_max*time_step)^2; //10% of the maximum linear displacement
Cnx(2,2) = (0.1*vx_max*time_step)^2; //10% of the maximum linear displacement
Cnx(3,3) = (0.1*wz_max*time_step)^2; //10% of the maximum angular displacement
Cnx(4,4) = (ax_max*time_step)^2; //full speed variation since prediction is at constant velocity
Cnx(5,5) = (wz_dot_max*time_step)^2; //full speed variation since prediction is at constant velocity

// compute the H matrix (constant for all iterations)
H = [zeros(3,3) [1/wheel_radius -base_length/(2*wheel_radius);1/wheel_radius base_length/(2*wheel_radius);0 1]];

// Identity will be useful in the loop
I = eye(5,5); //set a 5x5 Identity

// plot arrays
x_array = [x];
det_Cx_array = [det(Cx(4:5,4:5))];
//det_Cx_array = [det(Cx)];
x_true_array = [];
x_error = [];

// main loop
for t=1:n_iter

    // generate synthetic measurements with noise
    vx_true = vx_true_amp*sin( (2*%pi/vx_true_period)*t*time_step );
    wz_true = wz_true_amp*sin( (2*%pi/wz_true_period)*t*time_step );
    omega_l_true = (vx_true-wz_true*base_length/2)/wheel_radius;//I-kinematics
    omega_r_true = (vx_true+wz_true*base_length/2)/wheel_radius;//I-kinematics
    omega_l_measurement = omega_l_true+rand()*omega_noise;
    omega_r_measurement = omega_r_true+rand()*omega_noise;
    gyro_measurement = wz_true+rand()*gyro_noise;
    z = [omega_l_measurement;omega_r_measurement;gyro_measurement];

    //************* PREDICTION STEP **********************

    // state mean prediction
    x(1) = x_array(1,t) + x_array(4,t)*time_step*cos(x_array(3,t));
    x(2) = x_array(2,t) + x_array(4,t)*time_step*sin(x_array(3,t));
    x(3) = x_array(3,t) + x_array(5,t)*time_step;
    x(4) = x_array(4,t);
    x(5) = x_array(5,t);

    // state covariance prediction. First compute the Jacobian F
    F = [1 0 -x(4)*time_step*sin(x(3)) time_step*cos(x(3)) 0;
         0 1  x(4)*time_step*cos(x(3)) time_step*sin(x(3)) 0;
         0 0 1 0 time_step;
         0 0 0 1 0;
         0 0 0 0 1];
    Cx = F*Cx*F' + Cnx;
    //det_Cx_array = [det_Cx_array det(Cx(4:5,4:5))];
    det_Cx_array = [det_Cx_array det(Cx)];


    //************* CORRECTION STEP **********************

    // compute the expected measurement
    z_exp = H*x;

    // compute the K gain
    K = Cx*H'*inv(H*Cx*H'+Cnz);

    // update state mean and covariance
    x = x + K*(z-z_exp);
    Cx = (I-K*H)*Cx*(I-K*H)' + K*Cnz*K';

    // keep values for plotting
    x_array = [x_array x];
    //det_Cx_array = [det_Cx_array det(Cx(4:5,4:5))];
    det_Cx_array = [det_Cx_array det(Cx)];
    x_true_array = [x_true_array [vx_true;wz_true]];
    x_error = [x_error [(x(4)-vx_true);(x(5)-wz_true)]];

end

//plots
figure('BackgroundColor',[1 1 1]);
plot(x_true_array(1,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'vx TRUE, [m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_true_array(2,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'wz TRUE, [rad/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

//plots
figure('BackgroundColor',[1 1 1]);
plot(x_array(4,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Linear Velocity, v[m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(det_Cx_array(3:$));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'det(Cx)';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_error(1,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'error vx';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_error(2,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'error wz';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
