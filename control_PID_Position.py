from moteurCC import MoteurCC
from pylab import *

class ControlPID_Position(object):
    
    def __init__(self, motor, Kp=1.0, Ki=0.0, Kd=0.0):
        self.motor = motor
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        self.target = 0.0 # Cible en radians (Angle)
        
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_voltage = 0.0
        
        # Pour l'historique et les tracés
        self.target_history = [0.0] 
        self.error_history = [0.0]
    
    def setTarget(self, angle_rad):
        self.target = angle_rad
    
    def simule(self, step):
        # on récupère la position actuelle (et non la vitesse)
        current_pos = self.motor.getPosition()
        
        # calcul de l'erreur
        error = self.target - current_pos
        
        # terme Intégral (somme des erreurs)
        self.integral_error += error * step
        
        # terme Dérivé (variation de l'erreur)
        derivative_error = (error - self.previous_error) / step
        
        # calcul de la commande (Tension U)
        voltage = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)
        
        self.last_voltage = voltage
        self.previous_error = error
        
        # Application au moteur
        self.motor.setVoltage(voltage)
        self.motor.simule(step)
        
        self.target_history.append(self.target)
        self.error_history.append(error)

if __name__ == '__main__':
    
    # Paramètres du moteur
    R = 1.0
    L = 0.001
    ke = 0.01 
    kc = 0.01 
    J = 0.01
    f = 0.1
    
    # On crée 3 moteurs pour comparer les réglages
    m1 = MoteurCC(name="P Faible", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    m2 = MoteurCC(name="P Fort (Oscillations)", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    m3 = MoteurCC(name="P Fort + D (Correct)", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    
    # Cas 1 : juste un Kp faible
    pid1 = ControlPID_Position(m1, Kp=1.0, Ki=0.0, Kd=0.0)
    
    # Cas 2 : Un Kp fort
    pid2 = ControlPID_Position(m2, Kp=10.0, Ki=0.0, Kd=0.0)
    
    # Cas 3 : Kp fort (rapidité) + Kd (amortissement/freinage à l'approche).
    pid3 = ControlPID_Position(m3, Kp=10.0, Ki=0.0, Kd=1.0)
    
    target_pos = 1.0
    
    pid1.setTarget(target_pos)
    pid2.setTarget(target_pos)
    pid3.setTarget(target_pos)
    
    t = 0
    steps = 0.01
    temps = [t]
    
    while t < 5.0:
        t += steps
        temps.append(t)
        
        pid1.simule(steps)
        pid2.simule(steps)
        pid3.simule(steps)
    
    figure("Asservissement en Position")
    
    plot(temps, m1.position, label=f"P Faible (Kp={pid1.Kp})")
    plot(temps, m2.position, label=f"P Fort (Kp={pid2.Kp})")
    plot(temps, m3.position, label=f"P Fort + D (Kp={pid3.Kp}, Kd={pid3.Kd})", linewidth=2)
    
    # la consigne en pointillés noirs
    plot(temps, pid1.target_history, 'k--', label="Consigne")
    
    title("Influence des gains P et D sur la Position")
    xlabel('Temps (s)')
    ylabel('Position (rad)')
    grid(True)
    legend()
    show()