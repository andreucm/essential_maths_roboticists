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
rand("normal");//set the distribution type to the generator
sigma_far = 1*%pi/180; // [rad]
map = [[0;0] [0;3] [3;0] [3;3]]; // map of beacons

// init particle set
p_set = []; 
for ii=1:n_p
    px = x_0 + rand()*sigma_x0; 
    py = y_0 + rand()*sigma_y0; 
    pa = a_0 + rand()*sigma_a0;
    pw = 1/n_p; 
    p_set = [p_set [pw;px;py;pa] ]; //each column is a particle. First row weight, then x,y,theta
end

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
    

    // PREDICTION Loop, for each particle
    rand("normal");//set the distribution type to the normal generator
    for ii = 1:n_p
        // Compute twist+noise for each particle
        
        // Predict next pose for each particle
        // to do (exercice class 15)
    end
    

    // CORRECTION Loop, for each particle
    sum_wi = 0; 
    for ii = 1:n_p
        // For each measurement, k
        for kk=1:size(map)(2)
            ot = atan( map(2,kk)-y_0 , map(1,kk)-x_0 ) - a_0 + rand()*sigma_far; //real measurement with noise 
            oe = atan( map(2,kk)-p_set(3,ii) , map(1,kk)-p_set(2,ii) ) - p_set(4,ii); //expected measurement for particle ii
            L_oeot = erfc( abs(ot-oe)/(sqrt(2)*sigma_far) ); 
            p_set(1,ii) = p_set(1,ii)*L_oeot; 
        end
        sum_wi = sum_wi + p_set(1,ii); 
    end
    for ii = 1:n_p
        p_set(1,ii) = p_set(1,ii)/sum_wi; // normalize weights, so the sum of all is 1
    end
    
       
    // PLOT particle set after correction
    drawlater(); 
    plot(ph,p_set(2,:),p_set(3,:),".");
    ph.isoview="on"; // isoview mode
    ph.auto_clear="on";
    ph.auto_scale="off";
    ph.axes_visible = ["on","on","off"]
    ph.grid = [1,1];
    ph.tight_limits="on";
    ph.data_bounds=[0,0;8,8];
    drawnow(); 
    
    
    // RESAMPLING
    
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
        p_set_new = [p_set_new [1/n_p; p_set(2,jj)+rand()*0.03; p_set(3,jj)+rand()*0.03; p_set(4,jj)+rand()*0.01] ];         
    end
    p_set = p_set_new; 
    
    sleep(50); // Pause execution for 100 ms

end

   


