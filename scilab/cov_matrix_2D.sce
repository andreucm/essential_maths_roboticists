//user entries
mm = 0.3;
bb = 0.5;
noise_stdev = 0.3; //sqrt(noise_variance)

//create set S1
xx1 = [0:0.1:10]';
[nn cols] = size(xx1); //get set size
yy1 = mm*xx1 + bb + noise_stdev*rand(nn,1,"normal");

//create set S2
xx2 = noise_stdev*rand(nn,1,"normal");
yy2 = noise_stdev*rand(nn,1,"normal");

//compute means, each component, each set:
mx1 = sum(xx1)/nn;
my1 = sum(yy1)/nn;
mx2 = sum(xx2)/nn;
my2 = sum(yy2)/nn;

//compute covariance matrix explicitly
cxx1 = (xx1-mx1)'*(xx1-mx1)/(nn-1);
cyy1 = (yy1-my1)'*(yy1-my1)/(nn-1);
cxy1 = (xx1-mx1)'*(yy1-my1)/(nn-1);
cxx2 = (xx2-mx2)'*(xx2-mx2)/(nn-1);
cyy2 = (yy2-my2)'*(yy2-my2)/(nn-1);
cxy2 = (xx2-mx2)'*(yy2-my2)/(nn-1);

//compute covariance matrix with scilab call
C1 = cov(xx1,yy1);
C2 = cov(xx2,yy2);
