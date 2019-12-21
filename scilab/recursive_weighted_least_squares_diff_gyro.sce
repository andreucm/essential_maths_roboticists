// clear
clear;

// input params
time_step = 0.01; // algorithm loop rate [s]
base_length = 1;// base length [m]
wheel_radius = 0.4;// wheel radius [m]
std_dev_wheel_encoder = ((2*%pi)/1024)/time_step;//[rad/s]
std_dev_gyro = 0.01;//[rad/s]

// simulation params
n_iter = 200;//[#]
vx_true = 7; //[m/s]
wz_true = 0.1; //[rad/s]
omega_noise = std_dev_wheel_encoder; //std dev[rad/s]dev[rad]
gyro_noise = std_dev_gyro; //std dev[rad/s]

// initializations
//I-kinematics to compute true rear wheel rotation speed and true steering
omega_l_true = (vx_true-wz_true*base_length/2)/wheel_radius; 
omega_r_true = (vx_true+wz_true*base_length/2)/wheel_radius; 
x = [0.2;0.2]; 
Cx = [1 0; 0 1];
Cnz = [std_dev_wheel_encoder^2 0 0; 0 std_dev_wheel_encoder^2 0; 0 0 std_dev_gyro^2];
I = eye(2,2); //set a 2x2 Identity
rand("normal");//set the distribution type to the generator

// compute the H matrix (constant for all iterations)
H = [1/wheel_radius -base_length/(2*wheel_radius);1/wheel_radius base_length/(2*wheel_radius);0 1];

// plot arrays
x_array = [x];
det_Cx_array = [det(Cx)]; 


// main loop
for t=1:n_iter

    // generate synthetic measurements with noise
    omega_l_measurement = omega_l_true+rand()*omega_noise;
    omega_r_measurement = omega_r_true+rand()*omega_noise;
    gyro_measurement = wz_true+rand()*gyro_noise;
    z = [omega_l_measurement;omega_r_measurement;gyro_measurement];

    // compute the expected measurement
    z_exp = H*x; 

    // compute the K gain
    K = Cx*H'*inv(H*Cx*H'+Cnz); 

    // update state mean and covariance
    x = x + K*(z-z_exp);
    Cx = (I-K*H)*Cx*(I-K*H)' + K*Cnz*K'; 

    // keep values for plotting
    x_array = [x_array x];
    det_Cx_array = [det_Cx_array det(Cx)];

end

//plots
figure('BackgroundColor',[1 1 1]);
plot(x_array(1,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Linear Velocity, v[m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_array(2,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Rotational Velocity, v[m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(det_Cx_array(2:$));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'det(Cx)';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
