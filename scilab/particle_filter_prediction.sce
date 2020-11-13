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


    // PREDICTION Loop, for each particle
    rand("normal");//set the distribution type to the normal generator
    for ii = 1:n_p
        // 1. Compute twist+noise for each particle
        vxi = vx + rand()*sigma_vx;     
        vyi = vy + rand()*sigma_vy; 
        wzi = wz + rand()*sigma_wz;         
        
        // 2. Predict next pose for each particle
        // TO DO: exercise class 15
        
    end
    
       
    // PLOT particle set after correction
    drawlater(); 
    ph.auto_clear="on";
    plot(ph,p_set(2,:),p_set(3,:),"."); //particle set
    ph.auto_clear="off";
    plot(ph,x,y,"r+"); // robot true position
    ph.isoview="on"; // isoview mode
    ph.auto_scale="off";
    ph.axes_visible = ["on","on","off"]
    ph.grid = [1,1];
    ph.tight_limits="on";
    ph.data_bounds=[0,0;8,8];
    drawnow(); 
    
    
    sleep(50); // Pause execution for 100 ms

end

   


