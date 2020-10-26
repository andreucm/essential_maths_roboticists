// clear
clear;

// input params
time_step = 0.01; // algorithm loop rate [s]
base_length = 1;// base length [m]
wheel_radius = 0.4;// wheel radius [m]
std_dev_wheel_encoder = ((2*%pi)/1024)/time_step;//[rad/s]
std_dev_steering = (2*%pi)/1024;//[rad]
std_dev_gyro = 0.01;//[rad/s]

// simulation params
n_iter = 200;//[#]
vx_true = 7; //[m/s]
wz_true = 0.1; //[rad/s]
omega_noise = std_dev_wheel_encoder; //std dev[rad/s]dev[rad]
alpha_noise = std_dev_steering; //std 
gyro_noise = std_dev_gyro; //std dev[rad/s]

// initializations
//I-kinematics to compute true rear wheel rotation speed and true steering
omega_true = vx_true/wheel_radius; 
alpha_true = atan(wz_true*base_length/vx_true);
x = [3;0.2]; 
Cx = [1 0; 0 1];
Cnz = [std_dev_wheel_encoder^2 0 0; 0 std_dev_steering^2 0; 0 0 std_dev_gyro^2];
I = eye(2,2); //set a 2x2 Identity
rand("normal");//set the distribution type to the generator

// plot arrays
x_array = [x];

// main loop
for t=1:n_iter

    // generate synthetic measurements with noise
    omega_measurement = omega_true+rand()*omega_noise;
    alpha_measurement = alpha_true+rand()*alpha_noise;
    gyro_measurement = wz_true+rand()*gyro_noise;
    z = [omega_measurement;alpha_measurement;gyro_measurement];

    // compute the H matrix 
    wBv2 = (x(2)*base_length/x(1))^2;
    H = [1/wheel_radius 0; -x(2)*base_length/((1+wBv2)*x(1)^2) base_length/((1+wBv2)*x(1));0 1];

    // compute the expected measurement
    z_exp = H*x; 

    // compute the K gain
    K = Cx*H'*inv(H*Cx*H'+Cnz); 

    // update state mean and covariance
    x = x + K*(z-z_exp);
    Cx = (I-K*H)*Cx*(I-K*H)' + K*Cnz*K'; 
    x_array = [x_array x];
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
