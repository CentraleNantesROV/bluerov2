#!/usr/bin/env python

from pylab import *

class Model:
    def __init__(self, name,
                 Xdu, Ydv, Zdw, Kdp, Mdq, Ndr,
                 Xu, Yv, Zw, Kp, Mq, Nr,
                 Xuu, Yvv, Zww, Kpp, Mqq, Nrr):
        self.name = name
        self.Xdu = Xdu
        self.Ydv = Ydv
        self.Zdw = Zdw
        self.Kdp = Kdp
        self.Mdq = Mdq
        self.Ndr = Ndr
        
        self.Xu = Xu
        self.Yv = Yv
        self.Zw = Zw
        self.Kp = Kp
        self.Mq = Mq
        self.Nr = Nr
        self.Xuu = Xuu
        self.Yvv = Yvv
        self.Zww = Zww
        self.Kpp = Kpp
        self.Mqq = Mqq
        self.Nrr = Nrr
        
    def color(self, base):
        return base if self.name == 'classic' else base+'--'
        
    def plot(self, vel, axis):
        if axis == 'x':
            plot(vel, self.Xu*vel + self.Xuu*vel**2, self.color('C0'), label = self.name + ' ' + axis)
        if axis == 'y':
            plot(vel, self.Yv*vel + self.Yvv*vel**2, self.color('C1'), label = self.name + ' ' + axis)
        if axis == 'z':
            plot(vel, self.Zw*vel + self.Zww*vel**2, self.color('C2'), label = self.name + ' ' + axis)
        if axis == 'R':
            plot(vel, self.Kp*vel + self.Kpp*vel**2, self.color('C0'), label = self.name + ' ' + axis)
        if axis == 'P':
            plot(vel, self.Mq*vel + self.Mqq*vel**2, self.color('C1'), label = self.name + ' ' + axis)
        if axis == 'Y':
            plot(vel, self.Nr*vel + self.Nrr*vel**2, self.color('C2'), label = self.name + ' ' + axis)
                 
                 
# BlueROV2 heavy
# 6-DoF Modelling and Control of a Remotely Operated Vehicle
# Wu, Master thesis, Flinder University, 2018
heavy = Model('heavy', 5.5, 12.7, 14.57, .12, .12, .12, 
              4.03, 6.22, 5.18, .07, .07, .07, 
              18.18, 21.66, 36.99, 1.55, 1.55, 1.55)

# BlueROV2 classic
# Numerical Modelling and Experimental Testing of the Hydrodynamic Characteristics for an Open-Frame Remotely Operated Vehicle
# Li et al, Marine Science and Engineering, 2020
# no added mass
# no roll / pitch damping
classic = Model('classic', 5.5, 12.7, 14.57, .12, .12, .12, 
                1.31, 9.14, 2.015, .07, .07, 0,
                38.17, 129.6, 243.2, 1.55, 1.55, 4.86)


lin = linspace(0, 1, 100)
ang = linspace(0, 2, 100)

close('all')
for axis in 'xyz':
    heavy.plot(lin, axis)
    classic.plot(lin, axis)
legend()
    
figure()
for axis in 'RPY':
    heavy.plot(ang, axis)
    classic.plot(ang, axis)
legend()
    
show()








