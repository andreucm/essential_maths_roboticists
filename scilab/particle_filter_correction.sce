// particle filter prediction


// input params
x_0 = 1.5; 
y_0 = 2.5; 
a_0 = %pi/6; 
n_p = 200;
n_it = 150; 
dt = 0.1; 
sigma_x0 = 0.5; // [m]
sigma_y0 = 0.5; // [m]
sigma_a0 = 0.1; // [rad]
sigma_vx = 0.1; // [m/s] (1 cm/s)
sigma_vy = 0.1; // [m/s] (1 cm/s)
sigma_wz = 1*%pi/180; // [rad/s] (1º/s)
rand("normal");//set the distribution type to the generator
sigma_far_measurement = 1*%pi/180; // [rad]
sigma_far_likelihood = 2*%pi/180; // [rad]
map = [[0;0] [0;3] [3;0] [3;3] [2;5] [5;2] [6;6] [0;8] [8;8]]; // map of beacons

// init particle set
p_set = []; 
for ii=1:n_p
    px = x_0 + rand()*sigma_x0; 
    py = y_0 + rand()*sigma_y0; 
    pa = a_0 + rand()*sigma_a0;
    pw = 1/n_p; 
    p_set = [p_set [pw;px;py;pa] ]; //each column is a particle. First row weight, then x,y,theta
end

// init real robot pose
x = x_0; 
y = y_0; 
a = a_0; 

// init plot window
figure('BackgroundColor',[1 1 1]);
drawlater(); 
ph = gca(); // handle
plot(ph,p_set(2,:),p_set(3,:),".");
ph.isoview="on"; // isoview mode
ph.auto_clear="on";
ph.auto_scale="off";
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.tight_limits="on";
ph.data_bounds=[0,0;8,8];
drawnow(); 
sleep(500); // Pause execution for 500 ms

// loop for each iteration
for tt=1:n_it
    
    if tt < 21
        vx = 0.5; 
        vy = 0.5; 
        wz = 0; 
    end
    if tt>20 && tt<51
        vx = 0.3; 
        vy = 0.3; 
        wz = 0.2;         
    end
    if tt>50
        vx = 0.5; 
        vy = 0.1; 
        wz = -0.2;         
    end
    
    //update the pose of the real robot
    TM_p = [ cos(a) -sin(a) x;
             sin(a) cos(a) y;
             0 0 1];
    Td =  [cos(wz*dt) -sin(wz*dt) vx*dt;
           sin(wz*dt) cos(wz*dt) vy*dt;
           0 0 1];
    T = TM_p*Td; 
    x = T(1,3);
    y = T(2,3);        
    a = atan(T(2,1),T(1,1));


    // CORRECTION Loop, for each particle
    for ii = 1:n_p
        // For each measurement, k
        for kk=1:size(map)(2)
            // TO DO: Exercise class 17
            
        end
    end    
       
    // PLOT particle set after correction
    drawlater(); 
    ph.auto_clear="on";
    plot(ph,map(1,:),map(2,:),"go"); //beacons
    ph.auto_clear="off";
    plot(ph,p_set(2,:),p_set(3,:),"."); //particle set
    plot(ph,x,y,"r+"); // robot true position
    ph.isoview="on"; // isoview mode
    ph.auto_scale="off";
    ph.axes_visible = ["on","on","off"]
    ph.grid = [1,1];
    ph.tight_limits="on";
    ph.data_bounds=[0,0;8,8];
    drawnow(); 
    
    
    // RESAMPLING

    // normalize weights, so the sum of all is 1
    sum_wi = 0; 
    for ii = 1:n_p
        sum_wi = sum_wi + p_set(1,ii); 
    end
    for ii = 1:n_p
        p_set(1,ii) = p_set(1,ii)/sum_wi; 
    end
    
    // sort particle set according weigths (first particle, highest weight, and decreasing)
    p_set_sorted = gsort(p_set,'lc');
    
    // compute the cumulative_weight vector 
    cumulative_weight = zeros(1,n_p);
    cumulative_weight(1) = p_set_sorted(1,1); //first cumulative weight is equal to first weight
    for ii = 2:n_p //loop from the second particle
        cumulative_weight(ii) = cumulative_weight(ii-1)+p_set_sorted(1,ii); 
    end

    // draw a new particle set according weights of the current one    
    p_set_new = []; 
    for ii = 1:n_p
        rand("uniform");//set the distribution type to the uniform generator
        rv = rand();
        for jj = 1:n_p
            if rv < cumulative_weight(jj)
                break; 
            end
        end
        rand("normal"); 
        p_set_new = [p_set_new [1/n_p; p_set_sorted(2,jj)+rand()*0.02; p_set_sorted(3,jj)+rand()*0.02; p_set_sorted(4,jj)+rand()*0.01]];         
    end
    p_set = p_set_new; 
    
    sleep(50); // Pause execution for 100 ms

end

   


