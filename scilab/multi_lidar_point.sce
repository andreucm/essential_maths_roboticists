
//point in Lidar 1
q = [10;3;1];

//lidar motion
T1_1 = eye(3,3);
T2_1 = [cos(-%pi/8) -sin(-%pi/8) 0.1; sin(-%pi/8) cos(-%pi/8) 0 0.2; 0 0 1 -0.2; 0 0 0 1 ];
T3_1 = [cos(-%pi/5) -sin(-%pi/5) 0 0.5; sin(-%pi/5) cos(-%pi/5) 0 -0.1; 0 0 1 0.1; 0 0 0 1 ];
T4_1 = [cos(%pi/7) 0 -sin(%pi/7) 0.5; 0 1 0 0.1; sin(%pi/7) 0 cos(%pi/7) -0.1; 0 0 0 1 ];
T5_1 = [cos(%pi/9) 0 -sin(%pi/9) 0.4; 0 1 0 0.2; sin(%pi/9) 0 cos(%pi/9) 0.1; 0 0 0 1 ];

//measurements
u1 = K*T1_1*q + [rand(2,1);0];
//u1 = u1/u1(3) + [rand(2,1);0];
u2 = K*inv(T2_1)*q + [rand(2,1);0];
//u2 = u2/u2(3) + [rand(2,1);0];
u3 = K*inv(T3_1)*q + [rand(2,1);0];
//u3 = u3/u3(3) + [rand(2,1);0];
u4 = K*inv(T4_1)*q + [rand(2,1);0];
//u4 = u4/u4(3) + [rand(2,1);0];
u5 = K*inv(T5_1)*q + [rand(2,1);0];
//u5 = u5/u5(3) + [rand(2,1);0];

//mount the LS problem
z = [u1;u2;u3;u4;u5]; //measurements
H = [K*T1_1;K*inv(T2_1);K*inv(T3_1);K*inv(T4_1);K*inv(T5_1)]; //H matrix -> measurement model

//solve LS
x = inv(H'*H)*H'*z;
x = x/x(4);
disp(x);
