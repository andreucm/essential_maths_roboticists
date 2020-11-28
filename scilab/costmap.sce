
// input params
grid_resolution = 0.05; //cell size in meters
grid_size = 10; // grid side size in meters, so grid area is grid_size*grid_size 
map = [[2;3; 0.2] [-1;4; 0.3] [3;-2; 0.5] [-2;-2; 0.1]]; //map of circle-like obsatcles
robot_length = 0.5; //meters
robot_width = 0.3;  //meters
safety_margin = 0.1; //meters
lethal_cost = 255; //maximum cost for a costmap cell
decay_exp = 1; //cost decay is exp(-d*exp_decay), where d is the distance to obstacle+inflation+safety


// inits
n_cells = grid_size/grid_resolution; //number of cells of a single grid row or column. Total cells is n_cells*n_cells
grid_map = zeros (n_cells, n_cells); // occupancy grid representation of map
grid_costmap = zeros (n_cells, n_cells);

//build the grid map
for ii=1:1:n_cells
    for jj=1:1:n_cells
        //convert cell to point
        p_ij = [-(ii-1)*grid_resolution + grid_size/2; -(jj-1)*grid_resolution + grid_size/2]
        
        //compute cost of point, loop over map obstacles
        cost = 0; 
        for kk=1:1:size(map)(2)
            d = sqrt( (p_ij(1)-map(1,kk))*(p_ij(1)-map(1,kk)) + (p_ij(2)-map(2,kk))*(p_ij(2)-map(2,kk)) );
            if (d < map(3,kk))
                cost = cost + lethal_cost; 
            else
                cost = cost + 0; 
            end
        end
        
        //set cost to cell ij
        grid_map(ii,jj) = cost; 
    end
end
grid_map (1,1) = lethal_cost; 
grid_map (1,2) = lethal_cost; 

//show the grid map
figure('BackgroundColor',[1 1 1]);
drawlater(); 
ph = gca(); // handle
Matplot(-grid_map+lethal_cost); // inversion is to see free space in white, and occupied space in black
ph.isoview="on"; // isoview mode
//ph.data_bounds=[-5,-5;5,5];
ph.title.text = 'Local map (robot-centered)';
ph.x_label.text = 'y [m]';
ph.y_label.text = 'x [m]';
ph.x_ticks =  tlist(["ticks", "locations", "labels"], [1 100 200], ["-5" "0" "5"]);
ph.y_ticks =  tlist(["ticks", "locations", "labels"], [1 100 200], ["-5" "0" "5"]);
drawnow(); 
 
 
//build the grid costmap

//show the grid costmap
