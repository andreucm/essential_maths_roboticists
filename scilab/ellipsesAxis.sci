//computes ellipses minor and major axis from 2D covariance matrix
function [axis] = ellipsesAxis(Cmat)
    
    //ensure positive-definite matrix
    Cchol = chol(Cmat);
    CC = Cchol'*Cchol;  
    
    //compute evecs and evals
    [RR,diagCC] = spec(CC);
    eval1 = diagCC(1,1);
    eval2 = diagCC(2,2);
    evec1 = [RR(1,1);RR(2,1)];
    evec2 = [RR(1,2);RR(2,2)];
    
    //return first the biggest eval, then the smallest and finally the angle
    if (eval1>eval2) then
        axis = [eval1;eval2;atan(RR(2,1),RR(1,1))*180/%pi];
    else
        axis = [eval2;eval1;atan(RR(2,2),RR(1,2))*180/%pi];
    end
endfunction

//trace_ellipse
//xc,yc is the center
//a main axis, the one which is rotates phi wrt the plot horizontal
//b second axis
function [] = draw_ellipses(xc, yc, a, b, phi)
    step = 0.1;
    t = 0:step:%pi/2;
    X = a*cos(t);
    Y = b*sin(t);
    n = 4*size(X,'*');
    XY1 = [X, -flipdim(X,2), -X, flipdim(X,2);...
        Y, flipdim(Y,2), -Y, -flipdim(Y,2)];
    XY = rotate(XY1, phi) + [xc*ones(1,n);yc*ones(1,n)];
    xpoly(XY(1,:), XY(2,:));
endfunction 

//draws ellipses in the current axis, given by the 
//covariance matrix Cmat, and centered at point mu
function[] = draw_ellispes_from_cov(mu, Cmat, axes_h)

    //ensure positive-definite matrix
    Cchol = chol(Cmat);
    CC = Cchol'*Cchol;  
    
    //compute eigenvalues
    [RR,diagCC] = spec(CC);
    eval1 = diagCC(1,1);
    eval2 = diagCC(2,2);
    
    //sort evals by value. Set major and minor axes, and orientation angle of the ellipses
    if (eval1>eval2) then
        axis = [eval1;eval2;atan(RR(2,1),RR(1,1))];
    else
        axis = [eval2;eval1;atan(RR(2,2),RR(1,2))];
    end

    //start drawing (compute all points) 
    step = 0.1; //Set drawing step    
    t = 0:step:%pi/2; //set drawing vector
    eX = axis(1)*cos(t); //ellispes points (X component, a quarter)
    eY = axis(2)*sin(t); //ellipses points (Y component, a quarter)
    nn = 4*size(eX,'*'); //num of total points of the ellipses
    eXY1 = [eX, -flipdim(eX,2), -eX, flipdim(eX,2); eY, flipdim(eY,2), -eY, -flipdim(eY,2)];
    eXY = rotate(eXY1, axis(3)) + [mu(1)*ones(1,nn);mu(2)*ones(1,nn)];
    sca(axes_h); //Set current axes
    xpoly(eXY(1,:), eXY(2,:));

endfunction
