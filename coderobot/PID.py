#https://www.youtube.com/watch?v=chFNSg_cyKE

import numpy as np
import matplotlib.pyplot as plt 
import scipy.integrate as S

def Derivatives(z,t):
    x=z[0];
    xdot = z[1];
    eint = z[2];
    kp=50
    ki=10
    kd=25
    
    #%%% out input
    #%%% PROPORTIONAL CONTROL
    xc =10;
    e=xc-x;
    
    #%%% DERIVATIVE CONTROL
    xcdot=0;
    edot = xcdot-xdot;
    
    #%%% INTERAL GAIN
    
    #%%%% eint = integral(e)
    eintdot=e;
    f=kp*e + ki*eint + kd*edot;
    
    #%%%% this is the only thing that will change for second order systems
    xdbldot = f - 2*xdot -3*x
    
    zdot = np.asarray([xdot,xdbldot,eintdot]);
    
    return zdot

plt.close("all");


zinitial = np.asarray([0,0,0]);
tspan = np.arange(0,10,0.01);

zout = S.odeint(Derivatives,zinitial,tspan);

xout = zout[:,0];

plt.plot(tspan,xout)
plt.show()

