from vector3D import Vector3D as V3D
from particule import Particule
from univers import Univers
import matplotlib.pyplot as plt


class MoteurCC():
    
    def __init__(self, R=1.0, L=0.001, ke=0.01, kc=0.01, J=0.01, f=0.1, name="Motor1"):
        
        self.R = R              # Résistance (Ohms)
        self.L = L              # Inductance (Henry)
        self.ke = ke            # Constante de fcem (V.s/rad)
        self.kc = kc            # Constante de couple (N.m/A)
        self.J = J              # Inertie (kg.m^2)
        self.f = f              # Frottement visqueux (N.m.s/rad)
        self.name = name
        
        self.omega = [0.0]      # Vitesse de rotation (rad/s)
        self.i = [0.0]          # Courant (A)
        self.u_m = [0.0]        # Tension d'alimentation (V)
        self.gamma = [0.0]      # Couple moteur (N.m)
        self.position = [0.0]   # Position angulaire (rad)
        
    
    def __str__(self):
        return f"MoteurCC: {self.name} (R={self.R}, ke={self.ke}, J={self.J})"
    
    def __repr__(self):
        return str(self)
        
    def getPosition(self):
        return self.position[-1]
    
    def getSpeed(self):
        return self.omega[-1]
        
    def getTorque(self):
        return self.gamma[-1]
    
    def getIntensity(self):
        return self.i[-1]
    
    def setVoltage(self, U):
        self.u_m.append(U)
    
    def simule(self, step):
        omega_old = self.omega[-1]
        pos_old = self.position[-1]
        um_current = self.u_m[-1]
        
        # Calcul de E(t)
        e = self.ke * omega_old
        
        # Calcul du courant i(t) avec simplification L=0
        # Loi des mailles : Um = E + R*i => i = (Um - E) / R
        i_new = (um_current - e) / self.R
        
        gamma_m = self.kc * i_new
        
        # J * d(omega)/dt + f * omega = Gamma_m
        dw_dt = (gamma_m - self.f * omega_old) / self.J
        
        omega_new = omega_old + dw_dt * step
        pos_new = pos_old + omega_new * step 
        
        self.omega.append(omega_new)
        self.position.append(pos_new)
        self.i.append(i_new)
        self.gamma.append(gamma_m)
        
        if len(self.u_m) < len(self.omega):
            self.u_m.append(self.u_m[-1])
    
    def plot(self):
        from pylab import plot
        temps = [i * 0.01 for i in range(len(self.omega))]
        
        return plot(temps,self.omega)    

if __name__=='__main__':
    from pylab import figure, show, legend
    
    m = MoteurCC()
    
    print(m)
    
    t = 0
    step = 0.01
    temps = [t]
    
    while t<2 :
        t=t+step
        temps.append(t)
        m.setVoltage(1)
        m.simule(step)
    
    figure()
    m.plot()
    show()
