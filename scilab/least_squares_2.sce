
//data
u=[1;2;3;5];
z=[2;4;7;14];
H=[u(1)*u(1) u(1) 1;u(2)*u(2) u(2) 1;u(3)*u(3) u(3) 1;u(4)*u(4) u(4) 1]; 
Cnz=[0.1 0 0 0;0 1 0 0; 0 0 1 0; 0 0 0 0.1]; 

//least squares solving
s_ls = inv(H'*H)*H'*z;

//weighted least squares solving
s_wls = inv(H'*inv(Cnz)*H)*H'*inv(Cnz)*z;

//plot
r=[0:0.1:20];
plot(r, ( s_ls(1)*r.*r + s_ls(2)*r + s_ls(3) ), 'r' );
plot(r, ( s_wls(1)*r.*r + s_wls(2)*r + s_wls(3) ), 'g' );
plot(u, z, 'b.' );

