
//measurements
sigma_points = 0.01; 
points=[0 1 2 3 4 5 6 7 8 9 20 30 40 50 ;0 1 2 3 4 5 6 7 8 9 20 30 40 50 ];
points = points + sigma_points*rand(2,14,"normal");

//H matrix, b vector
H=points'; 
z = -ones(14,1);

//solving Least squares
x = inv(H'*H)*H'*z; 
disp(x);
