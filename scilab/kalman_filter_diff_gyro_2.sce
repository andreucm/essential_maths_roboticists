//______________________________________________________________________________

// A Kalman filter to estimate the state of a differential drive robot equipped
// with wheel encoders and one gyro.
// The state is [vx wz ax wzdot]^T  (m/s,rad/s,m/s2,rad/s2)
// The [vx,wz] components of the state are used to compute the odometry
//______________________________________________________________________________


// clear
clear;

// input params
time_step = 0.01; // algorithm loop rate [s]
base_length = 1;// base length [m]
wheel_radius = 0.4;// wheel radius [m]
std_dev_wheel_encoder = ((2*%pi)/4096)/time_step;//[rad/s]
std_dev_gyro = 0.01;//[rad/s]
//vx_max = 2; //[m/s]
//wz_max = %pi; //[rad/s]
ax_max = 5; //[m/s^2]
wz_dot_max = %pi; //[rad/s^2]
over_noise_factor = 5; // tunning: factor to increase or decrease Cnx according std_dev's, so we trust more or less in measurements

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
x = [0;0;0;0];
Cx = zeros(4,4);
Cx(1,1) = 1;
Cx(2,2) = 0.5^2;
Cx(3,3) = 1;
Cx(4,4) = 0.5^2;
Cnz = [over_noise_factor*std_dev_wheel_encoder^2 0 0; 0 over_noise_factor* std_dev_wheel_encoder^2 0; 0 0 over_noise_factor*std_dev_gyro^2];
Cnx = zeros(4,4);
Cnx(1,1) = (ax_max*time_step)^2;
Cnx(2,2) = (wz_dot_max*time_step)^2;
Cnx(3,3) = 1e-2;
Cnx(4,4) = 1e-2;

// compute the H matrix (constant for all iterations)
H = [ [1/wheel_radius -base_length/(2*wheel_radius);1/wheel_radius base_length/(2*wheel_radius);0 1]  zeros(3,2) ];

// compute the F matrix (constant for all iterations)
F = [1 0 time_step 0; 0 1 0 time_step; 0 0 1 0; 0 0 0 1];

// Identity will be useful in the loop
I = eye(4,4); //set a 4x4 Identity

// odom init
P = eye(3,3);

// plot arrays
x_array = [x];
//det_Cx_array = [det(Cx(4:5,4:5))];
det_Cx_array = [det(Cx)];
x_true_array = [];
x_error = [];
px_array = [];
py_array = [];

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

    //************* KF PREDICTION STEP **********************

    // state mean prediction
    x = F*x;

    // state covariance prediction.
    Cx = F*Cx*F' + Cnx;


    //************* KF CORRECTION STEP **********************

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
    x_error = [x_error [(x(1)-vx_true);(x(2)-wz_true)]];


    //*** TWIST INTEGRATION TO COMPUTE ODOMETRY **********
    dP = [  cos(x(2)*time_step) -sin(x(2)*time_step) (x(1)*time_step);
            sin(x(2)*time_step) cos(x(2)*time_step) 0;
            0 0 1 ];
    P = P*dP;
    px_array = [px_array P(1,3)];
    py_array = [py_array P(2,3)];

end

// **************** plots ********************

figure('BackgroundColor',[1 1 1]);
plot(x_true_array(1,:),"r");
plot(x_array(1,:),"b");
ph = gca(); // handle
ph.title.text = 'True vs Estimated linear velocity';
ph.x_label.text = 'iteration';
ph.y_label.text = 'Linear Velocity, v_x[m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_true_array(2,:),"r");
plot(x_array(2,:),"b");
ph = gca(); // handle
ph.title.text = 'True vs Estimated  rotational velocity';
ph.x_label.text = 'iteration';
ph.y_label.text = 'Rotational Velocity, w_z[rad/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(det_Cx_array(3:$));
ph = gca(); // handle
ph.title.text = 'Determinant of the estimated state covariance';
ph.x_label.text = 'iteration';
ph.y_label.text = 'det(Cx)';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_error(1,:));
ph = gca(); // handle
ph.title.text = 'Error of the estimated linear velocity';
ph.x_label.text = 'iteration';
ph.y_label.text = 'error vx [m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_error(2,:));
ph = gca(); // handle
ph.title.text = 'Error of the estimated rotational velocity';
ph.x_label.text = 'iteration';
ph.y_label.text = 'error wz [rad/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

//figure('BackgroundColor',[1 1 1]);
//plot(px_array(1,:),py_array(1,:));
//ph = gca(); // handle
//ph.x_label.text = 'positoin x';
//ph.y_label.text = 'position y';
//ph.axes_visible = ["on","on","off"]
//ph.grid = [1,1];
//ph.auto_scale="on";
