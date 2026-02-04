from moteurCC import MoteurCC
from pylab import *

class ControlPID_vitesse(object):
    
    def __init__(self, motor, Kp=1.0, Ki=0.0, Kd=0.0, Vmax=24.0):
        self.motor = motor
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.Vmax = Vmax
        
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
        
        integ = self.integral_error + error * step
        
        derivative_error = (error - self.previous_error) / step
        
        # commande brute théorique
        tension = (self.Kp * error) + (self.Ki * integ) + (self.Kd * derivative_error)
        
        # Saturation et Anti-Windup
        if tension > self.Vmax:
            voltage = self.Vmax
            # si l'erreur essaie encore d'augmenter la tension on refuse l'intégration
            if error > 0:
                self.integral_error = self.integral_error 
            else:
                self.integral_error = integ
                
        elif tension < -self.Vmax:
            voltage = -self.Vmax
            if error < 0:
                self.integral_error = self.integral_error 
            else:
                self.integral_error = integ
        else:
            # pas de saturation
            voltage = tension
            self.integral_error = integ
        
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
    
    P = 20.0
    I = 60.0
    D = 0.0
    
    control = ControlPID_vitesse(m_bf, Kp=P, Ki=I, Kd=D)
    
    # Calcul de la tension pour la boucle ouverte (1/K)
    # On veut que m_bo atteigne la même vitesse finale que la cible (1 rad/s)
    # Vitesse_inf = (U * kc) / (R*f + kc*ke)
    # Donc U_necessaire = Vitesse_cible * (R*f + kc*ke) / kc
    target_speed = 1.0 # rad/s
    
    denom = (R * f) + (kc * ke)
    inverse_gain_static = denom / kc
    voltage_bo = target_speed * inverse_gain_static
    
    print(f"\nGain statique inverse calculé : {inverse_gain_static:.4f}")
    print(f"Tension appliquée en BO pour atteindre {target_speed} rad/s : {voltage_bo:.2f} V")
    
    t = 0
    steps = 0.01
    temps = [t]
    
    while t < 3.0:
        t += steps
        temps.append(t)
        
        m_bo.setVoltage(voltage_bo) # Tension constante calculée
        
        control.setTarget(target_speed) # Consigne échelon
        control.simule(steps)
        
        m_bo.simule(steps)
    
    # On cherche la valeur absolue maximale dans l'historique des tensions
    u_max = max([abs(u) for u in control.voltage_history])
    print(f"\nPour Kp={P} et Ki={I} :")
    print(f"Tension MAX demandée par le PID = {u_max:.2f} V\n")
    
    figure("Comparaison BO vs BF")
    m_bo.plot()
    m_bf.plot()
    legend()
    show()