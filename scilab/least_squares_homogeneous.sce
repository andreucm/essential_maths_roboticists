
//data
xy=[1 2; 2 4; 3 7; 5 14]; //each row is a data point
D = [xy [1;1;1;1]];

//SVD decomposition
[U,S,V]=svd(D); 

// last column of V is least squares solution for this model and data
p = V(:,$);
v = [-p(2);p(1)];

//plot
r=[-20:0.1:20];
plot(v(1)*r,v(2)*r, 'r' );
plot(xy(:,1),xy(:,2), 'b.' );
