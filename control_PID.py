from moteurCC import MoteurCC
from pylab import *

class ControlPID_vitesse(object):
    
    def __init__(self, motor, Kp=1.0, Ki=0.0, Kd=0.0):
        self.motor = motor
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.target = 0.0
        
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_voltage = 0.0
        
        self.target_history = [0.0] 
        self.voltage_history = [0.0]

    def __str__(self):
        return f"PID Controler (Kp={self.Kp}, Ki={self.Ki}, Kd={self.Kd}) on {self.motor.name}"

    def __repr__(self):
        return str(self)

    def setTarget(self, speed):
        self.target = speed

    def getVoltage(self):
        return self.last_voltage

    def simule(self, step):
        current_speed = self.motor.getSpeed()
        error = self.target - current_speed
        
        self.integral_error += error * step
        derivative_error = (error - self.previous_error) / step
        
        # Calcul de la commande (Tension)
        # U = Kp*e + Ki*int(e) + Kd*de/dt
        voltage = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)
        
        self.last_voltage = voltage
        self.previous_error = error
        
        self.motor.setVoltage(voltage)
        self.motor.simule(step)
        
        self.target_history.append(self.target)
        self.voltage_history.append(voltage)

if __name__ == '__main__':
    
    R = 1.0
    L= 0.001
    ke= 0.01 
    kc=0.01 
    J=0.01
    f=0.1
    
    m_bo = MoteurCC(name="Boucle Ouverte", R = R, L= L, ke=ke, kc=kc, J=J, f=f)
    m_bf = MoteurCC(name="Boucle Fermée", R = R, L= L, ke=ke, kc=kc, J=J, f=f)
    
    P = 5.0
    I = 20.0
    D = 0.0
    
    control = ControlPID_vitesse(m_bf, Kp=P, Ki=I, Kd=D)
    
    # Calcul de la tension pour la boucle ouverte (1/K)
    # On veut que m_bo atteigne la même vitesse finale que la cible (1 rad/s)
    # Vitesse_inf = (U * kc) / (R*f + kc*ke)
    # Donc U_necessaire = Vitesse_cible * (R*f + kc*ke) / kc
    target_speed = 10.0 # rad/s
    
    denom = (R * f) + (kc * ke)
    inverse_gain_static = denom / kc
    voltage_bo = target_speed * inverse_gain_static
    
    print(f"Gain statique inverse calculé : {inverse_gain_static:.4f}")
    print(f"Tension appliquée en BO pour atteindre {target_speed} rad/s : {voltage_bo:.2f} V")
    
    t = 0
    steps = 0.01
    temps = [t]
    
    while t < 2.0:
        t += steps
        temps.append(t)
        
        m_bo.setVoltage(voltage_bo) # Tension constante calculée
        
        control.setTarget(target_speed) # Consigne échelon
        control.simule(steps)
        
        m_bo.simule(steps)
    
    figure("Comparaison BO vs BF")
    m_bo.plot()
    m_bf.plot()
    legend()
    show()