

std_dev = 2; //in [cm]. 
x = [-10:0.1:10];
f = erfc(abs(x/(std_dev*sqrt(2)))); 

figure('BackgroundColor',[1 1 1]);
plot(x,f);
ph = gca(); // handle
ph.x_label.text = 'O^t_L-O^e_L';
ph.y_label.text = 'L(O^t_L)';
ph.axes_visible = ["on","on","off"]
ph.grid = [1,1];
ph.auto_scale="on";
