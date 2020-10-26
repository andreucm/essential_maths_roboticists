// Kalman filter
// Estimation of the 2D twist of a 3-wheeled omnidirectional platform, 
// equipped with an incremental encoder per wheel and a gyroscope onboard 
// the platform.


// input params
n_iterations = 1000; 
L = 0.3; // distance from wheel to robot center [m]
dt = 0.01; // filter time step [s] 
encoder_std_dev = 0.5; // standard deviation of encoder measurements [rad/s]
encoder_cross_covariance = 0.01*0.01; // cross covariance between encoder measurements [rad^2/s^2]
encoder_std_dev_measurement = 0.2; // to generate noisy measurements
gyro_std_dev = 0.1; // standard deviation of gyro measurements [rad/s]
gyro_std_dev_measurement = 0.1; // to generate noisy measurements
initial_vx = 3; // initial vx [m/s]
initial_vy = -3; // initial vy [m/s]
initial_wz = -2; // initial wz [m/s]
initial_Cx_vx = 1; // initial covariance for vx [m^2/s^2]
initial_Cx_vy = 1; // initial covariance for vy [m^2/s^2]
initial_Cx_wz = 1; // initial covariance for wz [m^2/s^2]
prediction_model_covariance_vx = 0.1*0.1;
prediction_model_covariance_vy = 0.1*0.1;
prediction_model_covariance_wz = 0.1*0.1;
vx_true_amp = 2; //[m/s]
vx_true_t = 10; //[s]
vx_noise_std_dev = 0.1; //[]
vy_true_amp = 1; //[m/s]
vy_true_t = 5; //[s]
wz_true_amp = 0.2; //[m/s]
wz_true_t = 20; //[s]


//----------------------------------------------------------------
// inits
I = eye(3,3); //set a 4x4 Identity
x = [initial_vx;initial_vy;initial_wz];
Cx = [initial_Cx_vx 0 0; 0 initial_Cx_vy 0; 0 0 initial_Cx_wz];
F = [1 0 0; 0 1 0; 0 0 1];
H = [0 1 L; -sqrt(3)/2 -0.5 L; sqrt(3)/2 0.5 L; 0 0 1];
Cnx = [prediction_model_covariance_vx 0 0; 0 prediction_model_covariance_vy 0; 0 0 prediction_model_covariance_wz];
Cnz = [encoder_std_dev*encoder_std_dev encoder_cross_covariance encoder_cross_covariance 0;
       encoder_cross_covariance encoder_std_dev*encoder_std_dev encoder_cross_covariance 0;
       encoder_cross_covariance encoder_cross_covariance encoder_std_dev*encoder_std_dev 0;
       0 0 0  gyro_std_dev*gyro_std_dev];
x_plot = [x]; 
detCx_plot = [det(Cx)]; 
       
aux_plot = [];

// main loop
for t=1:n_iterations
    
    // 1. generate noisy measurements
    vx_true = vx_true_amp*sin( (2*%pi/vx_true_t)*t*dt );
    vy_true = vy_true_amp*sin( (2*%pi/vy_true_t)*t*dt );
    wz_true = wz_true_amp*sin( (2*%pi/wz_true_t)*t*dt );
    measurements_true = [H;[0 0 1]]*[vx_true;vy_true;wz_true]; 
    z(1) = measurements_true(1) + rand()*encoder_std_dev_measurement;
    z(2) = measurements_true(2) + rand()*encoder_std_dev_measurement;
    z(3) = measurements_true(3) + rand()*encoder_std_dev_measurement;
    z(4) = measurements_true(4) + rand()*gyro_std_dev_measurement;
       
    // 2. Kalman filter iteration
        // 2a. Prediction
        x = F*x; //state mean
        Cx = F*Cx*F' + Cnx; //state covariance
        
        // 2b. Correction     
        z_exp = H*x; //compute the expected measurement
        K = Cx*H'*inv(H*Cx*H'+Cnz); //compute the K gain
        x = x + K*(z-z_exp); // update state mean
        Cx = (I-K*H)*Cx*(I-K*H)' + K*Cnz*K'; // update state covariance
        
    // 3. Keep output results
    //aux_plot = [aux_plot wz_true];
    x_plot = [x_plot x]; 
    detCx_plot = [detCx_plot det(Cx)]; 


// end main loop    
end


// plot results
figure('BackgroundColor',[1 1 1]);
plot(x_plot(1,:));

figure('BackgroundColor',[1 1 1]);
plot(x_plot(2,:));

figure('BackgroundColor',[1 1 1]);
plot(x_plot(3,:));

figure('BackgroundColor',[1 1 1]);
plot(detCx_plot(10:$));


