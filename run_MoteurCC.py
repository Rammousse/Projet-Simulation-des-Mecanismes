from moteurCC import MoteurCC
from control_PID_Vitesse import ControlPID_vitesse
import matplotlib.pyplot as plt

def simulateAll(Kp, Ki, Kd, duration=5.0, target=10.0):
    # création d'un nouveau moteur (pour partir de 0 à chaque fois)
    motor = MoteurCC(name=f"PID(P={Kp}, I={Ki})", 
                    R=1.0, L=0.001, ke=0.01, kc=0.01, J=0.01, f=0.1)
    
    # création du contrôleur
    pid = ControlPID_vitesse(motor, Kp=Kp, Ki=Ki, Kd=Kd)
    
    t = 0
    step = 0.01
    times = [t]
    speeds = [0.0]
    
    while t < duration:
        t += step
        times.append(t)
        pid.setTarget(target)
        pid.simule(step)      # Calcule la commande et l'applique
        speeds.append(motor.getSpeed())
        
    # On cherche la valeur maximale atteinte
    tension_max = 0
    for u in pid.voltage_history:
        if abs(u) > tension_max:
            tension_max = abs(u)
            
    # On compare avec la limite du PID
    if tension_max >= (pid.Vmax - 0.01):
        print(f" Config Kp={Kp:5.1f}, Ki={Ki:5.1f} : SATURATION (Max: {tension_max:.2f} V)")
    else:
        print(f" Config Kp={Kp:5.1f}, Ki={Ki:5.1f} : OK         (Max: {tension_max:.2f} V)")
    
    return times, speeds

if __name__ == '__main__':
    target_speed = 1.0 # Consigne (rad/s)
    
    # Influence du Gain Proportionnel (P)
    gains_P = [0.5, 2.0, 10.0, 20.0, 150.0]
    
    plt.figure(figsize=(10, 6))
    for p in gains_P:
        t, v = simulateAll(Kp=p, Ki=0.0, Kd=0.0, target=target_speed)
        plt.plot(t, v, label=f'Kp={p}, Ki=0')
        
    plt.axhline(y=target_speed, color='r', linestyle='--', label='Consigne')
    plt.title("Influence du Gain Proportionnel (P) sur la vitesse")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (rad/s)")
    plt.grid(True)
    plt.legend()
    
    # Influence du Gain Intégral (I)
    fixed_P = 20.0
    gains_I = [0.0, 5.0, 15.0, 30.0, 60.0]
    
    plt.figure(figsize=(10, 6))
    for i in gains_I:
        t, v = simulateAll(Kp=fixed_P, Ki=i, Kd=0.0, target=target_speed)
        plt.plot(t, v, label=f'Kp={fixed_P}, Ki={i}')
        
    plt.axhline(y=target_speed, color='r', linestyle='--', label='Consigne')
    plt.title(f"Influence du Gain Intégral (I) avec Kp={fixed_P} fixe")
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (rad/s)")
    plt.grid(True)
    plt.legend()
    
    plt.show()