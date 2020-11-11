// particle filter prediction


// input params
x_0 = 1; 
y_0 = 1; 
a_0 = %pi/6; 
n_p = 200;
n_it = 150; 
dt = 0.1; 
sigma_x0 = 0.5; // [m]
sigma_y0 = 0.5; // [m]
sigma_a0 = 0.1; // [rad]
rand("normal");//set the distribution type to the generator
sigma_vx = 0.1; // [m/s] (1 cm/s)
sigma_vy = 0.1; // [m/s] (1 cm/s)
sigma_wz = 10*%pi/180; // [rad/s] (1º/s)


// init particle set
p_set = []; 
for ii=1:n_p
    px = x_0 + rand()*sigma_x0; 
    py = y_0 + rand()*sigma_y0; 
    pa = a_0 + rand()*sigma_a0;
    p_set = [p_set [px;py;pa] ];
end

// init plot window
figure('BackgroundColor',[1 1 1]);
drawlater(); 
ph = gca(); // handle
plot(ph,p_set(1,:),p_set(2,:),".");
ph.isoview="on"; // isoview mode
ph.auto_clear="on";
ph.auto_scale="off";
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.tight_limits="on";
ph.data_bounds=[0,0;8,8];
drawnow(); 
sleep(500); // Pause execution for 500 ms

// loop (only prediction)
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
    for ii = 1:n_p
        // 1. Compute twist+noise for each particle
        vxi = vx + rand()*sigma_vx;     
        vyi = vy + rand()*sigma_vy; 
        wzi = wz + rand()*sigma_wz;         
        
        // 2. Predict next pose for each particle
        TM_pi = [cos(p_set(3,ii)) -sin(p_set(3,ii)) p_set(1,ii);
                 sin(p_set(3,ii)) cos(p_set(3,ii)) p_set(2,ii);
                 0 0 1];
        Td =  [cos(wzi*dt) -sin(wzi*dt) vxi*dt;
               sin(wzi*dt) cos(wzi*dt) vyi*dt;
               0 0 1];
        T = TM_pi*Td; 
        p_set(1,ii) = T(1,3);
        p_set(2,ii) = T(2,3);        
        p_set(3,ii) = atan(T(2,1),T(1,1));
    end

    // plots
    drawlater(); 
    plot(ph,p_set(1,:),p_set(2,:),".");
    ph.isoview="on"; // isoview mode
    ph.auto_clear="on";
    ph.auto_scale="off";
    ph.axes_visible = ["on","on","off"]
    ph.grid = [1,1];
    ph.tight_limits="on";
    ph.data_bounds=[0,0;8,8];
    drawnow(); 

    sleep(100); // Pause execution for 500 ms
    //disp(tt)

end

   


