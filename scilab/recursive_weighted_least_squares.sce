//clear
clear;

//set number of iterations
n_iter = 200; 

//vehicle axis width
bb = 2;

//set vehicle speeds
v_true = 2.17; //true linear vehicle speed[m/s]
w_true = 0.12; //true rotational vehicle speed [rad/s]

//above vehicle linear and rot speeds, causes true left and right wheel linear speeds:
v_l_true = v_true + w_true*bb/2;
v_r_true = v_true - w_true*bb/2;

////Generate noisy measurements, for each of the four encoders, at each iteration
//Measurements are each wheel linear velocity, arranged as: z=[v_lf;v_lb;v_rf;v_rb] (l:left, r:right, f:front, b:back)
sigma_v_f = 0.05; //std dev of linear wheel speed, for front encoders [m/s]. They are cheaper than back ones!
sigma_v_b = 0.01; //std dev of linear wheel speed, for back encoders [m/s]. They are better than front ones!
rand("normal");//set the distribution type to the generator
v_lf = v_l_true + rand(1,n_iter)*sigma_v_f;//left front measurements
v_lb = v_l_true + rand(1,n_iter)*sigma_v_b;//left back measurements
v_rf = v_r_true + rand(1,n_iter)*sigma_v_f;//right front measurements
v_rb = v_r_true + rand(1,n_iter)*sigma_v_b;//right back measurements
z = [v_lf;v_lb;v_rf;v_rb]; //stack all measurements in a single matrix

//****************** Recursive Least Squares starts here. ************************

//initial guess for the state: [v w]
x_est = [0;0];

//initial guess for the state covariance
C_x = [1 0; 0 0.2];

//measurement model (constant)
H = [1 bb/2; 1 bb/2; 1 -bb/2; 1 -bb/2];

//sets measurement process covariance (use values of sigma previously used to generate data)
sigma2_v_fb = 0.005^2; //covariance front-back linear wheel speeds [m^2/s^2]
C_nz = [sigma_v_f^2 sigma2_v_fb sigma2_v_fb sigma2_v_fb;
          sigma2_v_fb sigma_v_b^2 sigma2_v_fb sigma2_v_fb;
          sigma2_v_fb sigma2_v_fb sigma_v_f^2 sigma2_v_fb;
          sigma2_v_fb sigma2_v_fb sigma2_v_fb sigma_v_b^2];
          
//init other usefuls matrixes
I = eye(2,2); //set a 2x2 Identity
x_est_array = []; //state estimate at each iteration
tr_C_x_array = []; //trace of state covariance matrix

//recursive loop
for ii=1:n_iter
    
    //compute recursive weighted linear least squares
    z_exp = H*x_est; //expected measurememt
    K = C_x*H'*inv(H*C_x*H'+C_nz); //gain
    x_est = x_est + K*(z(:,ii)-z_exp); //update state estimate
    C_x = (I-K*H)*C_x*(I-K*H)' + K*C_nz*K'; //update state covariance
    
    //collect results
    x_est_array = [x_est_array x_est]; 
    tr_C_x_array = [tr_C_x_array trace(C_x)];
end

//plots
figure('BackgroundColor',[1 1 1]);
plot(x_est_array(1,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Linear Velocity, v[m/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(x_est_array(2,:));
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Rotational Velocity, w[rad/s]';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";

figure('BackgroundColor',[1 1 1]);
plot(tr_C_x_array);
ph = gca(); // handle
ph.x_label.text = 'iteration';
ph.y_label.text = 'Trace(C_x)';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
