from moteurCC import MoteurCC
from pylab import *

class ControlPID_Position(object):
    
    def __init__(self, motor, Kp=1.0, Ki=0.0, Kd=0.0, Vmax=24.0):
        self.motor = motor
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.Vmax = Vmax
        
        self.target = 0.0 # Cible en radians (Angle)
        
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.last_voltage = 0.0
        
        # Pour l'historique et les tracés
        self.target_history = [0.0] 
        self.error_history = [0.0]
        self.voltage_history = [0.0]
    
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
        
        # Calcul de la commande théorique (sans saturation)
        voltage_theo = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative_error)
        
        # Gestion de la Saturation et Anti-Windup
        if voltage_theo > self.Vmax:
            voltage = self.Vmax
            if error < 0: # Si on sature mais qu'on veut revenir, on intègre
                self.integral_error = self.integral_error
        elif voltage_theo < -self.Vmax:
            voltage = -self.Vmax
            if error > 0: 
                self.integral_error = self.integral_error
        else:
            voltage = voltage_theo
            self.integral_error = self.integral_error
        
        self.last_voltage = voltage
        self.previous_error = error
        
        # Application au moteur
        self.motor.setVoltage(voltage)
        self.motor.simule(step)
        
        self.target_history.append(self.target)
        self.error_history.append(error)
        self.voltage_history.append(voltage)

if __name__ == '__main__':
    
    # Paramètres du moteur
    R = 1.0
    L = 0.001
    ke = 0.01 
    kc = 0.01 
    J = 0.01
    f = 0.1

    target_pos = 1.0
    
    # Influence de Kp seul (Kd = 0)
    m_kp1 = MoteurCC(name="Moteur Kp1", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kp1 = ControlPID_Position(m_kp1, Kp=1.0, Ki=0.0, Kd=0.0)
    
    m_kp2 = MoteurCC(name="Moteur Kp2", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kp2 = ControlPID_Position(m_kp2, Kp=10.0, Ki=0.0, Kd=0.0)
    
    m_kp3 = MoteurCC(name="Moteur Kp3", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kp3 = ControlPID_Position(m_kp3, Kp=20.0, Ki=0.0, Kd=0.0)
    
    m_kp4 = MoteurCC(name="Moteur Kp4", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kp4 = ControlPID_Position(m_kp4, Kp=50.0, Ki=0.0, Kd=0.0)
    
    m_kp5 = MoteurCC(name="Moteur Kp5", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kp5 = ControlPID_Position(m_kp5, Kp=100.0, Ki=0.0, Kd=0.0)


    # Influence de Kd seul
    kp_fixe = 100.0     # On fixe Kp=200 car

    m_kd1 = MoteurCC(name="Moteur Kd1", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kd1 = ControlPID_Position(m_kd1, Kp=kp_fixe, Ki=0.0, Kd=0.0)
    
    m_kd2 = MoteurCC(name="Moteur Kd2", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kd2 = ControlPID_Position(m_kd2, Kp=kp_fixe, Ki=0.0, Kd=5.0)
    
    m_kd3 = MoteurCC(name="Moteur Kd3", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kd3 = ControlPID_Position(m_kd3, Kp=kp_fixe, Ki=0.0, Kd=10.0)
    
    m_kd4 = MoteurCC(name="Moteur Kd4", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kd4 = ControlPID_Position(m_kd4, Kp=kp_fixe, Ki=0.0, Kd=50.0)
    
    m_kd5 = MoteurCC(name="Moteur Kd5", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_kd5 = ControlPID_Position(m_kd5, Kp=kp_fixe, Ki=0.0, Kd=100.0)


    # Comparaison des configurations "Idéales" / Caractéristiques
    
    m_c1 = MoteurCC(name="P Moyen", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_c1 = ControlPID_Position(m_c1, Kp=20.0, Ki=0.0, Kd=0.0)
    
    m_c2 = MoteurCC(name="P Fort Instable", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_c2 = ControlPID_Position(m_c2, Kp=100.0, Ki=0.0, Kd=10.0)
    
    m_c3 = MoteurCC(name="Optimal (PD)", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_c3 = ControlPID_Position(m_c3, Kp=200.0, Ki=0.0, Kd=15.0)

    m_c4 = MoteurCC(name="Sur-amorti", R=R, L=L, ke=ke, kc=kc, J=J, f=f)
    pid_c4 = ControlPID_Position(m_c4, Kp=500.0, Ki=0.0, Kd=30.0)

    # Groupe 1
    for pid in [pid_kp1, pid_kp2, pid_kp3, pid_kp4, pid_kp5]: pid.setTarget(target_pos)
    # Groupe 2
    for pid in [pid_kd1, pid_kd2, pid_kd3, pid_kd4, pid_kd5]: pid.setTarget(target_pos)
    # Groupe 3
    for pid in [pid_c1, pid_c2, pid_c3, pid_c4]: pid.setTarget(target_pos)

    # Simulation
    t = 0
    steps = 0.01
    temps = [t]
    
    while t < 5.0:
        t += steps
        temps.append(t)
        
        pid_kp1.simule(steps); pid_kp2.simule(steps); pid_kp3.simule(steps)
        pid_kp4.simule(steps); pid_kp5.simule(steps)
        
        pid_kd1.simule(steps); pid_kd2.simule(steps); pid_kd3.simule(steps)
        pid_kd4.simule(steps); pid_kd5.simule(steps)
        
        pid_c1.simule(steps); pid_c2.simule(steps)
        pid_c3.simule(steps); pid_c4.simule(steps)
    
    
    # ANALYSE DES TENSIONS / SATURATION
    
    def check_saturation(pid, label):
        v_max = max([abs(v) for v in pid.voltage_history])
        etat = "OK"
        if v_max >= (pid.Vmax - 0.01):
            etat = "SATURATION"
        print(f"[{label:20s}] Kp={pid.Kp:5.1f}, Kd={pid.Kd:4.1f} : {etat:10s} (Max: {v_max:.2f} V)")

    print("\n--- GROUPE 1 : Influence Kp ---")
    check_saturation(pid_kp1, "P Tres Faible")
    check_saturation(pid_kp2, "P Moyen")
    check_saturation(pid_kp3, "P Rapide")
    check_saturation(pid_kp4, "P Fort")
    check_saturation(pid_kp5, "P Tres Fort")
    
    print("\n--- GROUPE 2 : Influence Kd (avec Kp=200) ---")
    check_saturation(pid_kd1, "Sans Derive")
    check_saturation(pid_kd2, "D Faible")
    check_saturation(pid_kd3, "D Moyen")
    check_saturation(pid_kd4, "D Optimal")
    check_saturation(pid_kd5, "D Trop Fort")

    print("\n--- GROUPE 3 : configurations Idéales---")
    check_saturation(pid_kd1, "Sans Derive")
    check_saturation(pid_kd2, "D Faible")
    check_saturation(pid_kd3, "D Moyen")
    check_saturation(pid_kd4, "D Optimal")
    check_saturation(pid_kd5, "D Trop Fort")
    
    # GRAPHIQUES

    # Influence de Kp
    figure("Influence de Kp (Kd=0)")
    plot(temps, m_kp1.position, label=f"Kp={pid_kp1.Kp}")
    plot(temps, m_kp2.position, label=f"Kp={pid_kp2.Kp}")
    plot(temps, m_kp3.position, label=f"Kp={pid_kp3.Kp}")
    plot(temps, m_kp4.position, label=f"Kp={pid_kp4.Kp}")
    plot(temps, m_kp5.position, label=f"Kp={pid_kp5.Kp}")
    plot(temps, pid_kp1.target_history, 'k--', label="Consigne")
    
    title("Influence du Gain Proportionnel Kp (Kd=0)")
    xlabel('Temps (s)')
    ylabel('Position (rad)')
    grid(True)
    legend(loc='lower right')
    
    # Influence de Kd
    figure("Influence de Kd (Kp=200)")
    plot(temps, m_kd1.position, label=f"Kd={pid_kd1.Kd}")
    plot(temps, m_kd2.position, label=f"Kd={pid_kd2.Kd}")
    plot(temps, m_kd3.position, label=f"Kd={pid_kd3.Kd}")
    plot(temps, m_kd4.position, label=f"Kd={pid_kd4.Kd}", linewidth=2)
    plot(temps, m_kd5.position, label=f"Kd={pid_kd5.Kd}")
    plot(temps, pid_kd1.target_history, 'k--', label="Consigne")
    
    title(f"Influence du Gain Dérivé Kd (avec Kp={kp_fixe} fixe)")
    xlabel('Temps (s)')
    ylabel('Position (rad)')
    grid(True)
    legend(loc='lower right')

    # Synthèse / Comparaison
    figure("Synthèse : Comparaison des configurations")
    plot(temps, m_c1.position, label=f"Kp={pid_c1.Kp}, Kd={pid_c1.Kd}")
    plot(temps, m_c2.position, label=f"Kp={pid_c2.Kp}, Kd={pid_c2.Kd}")
    plot(temps, m_c3.position, label=f"Kp={pid_c3.Kp}, Kd={pid_c3.Kd}", linewidth=2)
    plot(temps, m_c4.position, label=f"Kp={pid_c4.Kp}, Kd={pid_c4.Kd}")
    plot(temps, pid_c1.target_history, 'k--', label="Consigne")
    
    title("Comparaison : P seul vs PD optimal vs Sur-amorti")
    xlabel('Temps (s)')
    ylabel('Position (rad)')
    grid(True)
    legend(loc='lower right')
    
    show()