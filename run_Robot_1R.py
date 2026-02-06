import pygame
from math import pi, atan2
from types import MethodType
import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, LiaisonPivot, LiaisonMotorisee
from particule import Particule
from barre2D import Barre2D
from moteurCC import MoteurCC
from control_PID_Robot import ControlPIDRobot

def simulate_response(Kp, Ki, Kd, duration=3.0):
    """
    Simule le comportement du robot pour un jeu de gains donné sur une durée définie.
    Retourne les tableaux de temps et d'angle.
    """
    dt = 0.001 
    monde = Univers(name="Simu_Test", step=dt)
    
    # Création du Robot identique à la démo
    pivot_pos = V3D(0, 0)
    pivot = Particule(p0=pivot_pos, fix=True, name="Pivot")
    L_bras = 1.0
    # On initialise le bras vers le bas (-pi/2)
    bras = Barre2D(mass=0.2, long=L_bras, theta=-pi/2, pos=pivot_pos + V3D(0, -L_bras/2), nom="Bras")
    
    # Liaison mécanique
    liaison_eca = LiaisonPivot(pivot, bras, anchor1=-1, k=20000, c=50)
    
    # Moteur et PID à tester
    moteur = MoteurCC(R=2.0, ke=0.5, kc=0.5, name="MoteurTest") 
    pid = ControlPIDRobot(Kp=Kp, Ki=Ki, Kd=Kd, output_limit=24.0)
    
    # On part de -0.5 rad et on demande d'aller à +0.5 rad (échelon)
    bras.theta = -0.5 
    bras.omega = 0
    pid.target = 1.0
    
    target_val = pid.target
    
    # Liaison motorisée
    liaison_mot = LiaisonMotorisee(pivot, bras, moteur, pid, dt, anchorA=0, anchorB=-1)
    
    monde.addParticule(pivot, bras)
    monde.addGenerators(Gravity(V3D(0, -9.81)), liaison_eca, liaison_mot)
    
    t_vals = []
    theta_vals = []
    
    steps = int(duration / dt)
    for _ in range(steps):
        monde.simulateAll()
        t_vals.append(monde.time[-1])
        # Normalisation de l'angle pour affichage propre
        angle = (bras.theta + pi) % (2 * pi) - pi
        theta_vals.append(angle)
        
    return t_vals, theta_vals, target_val


def analyze_controllers():
    print("Calcul des courbes de réponse en cours...")
    
    # Influence du Gain Proportionnel (P)
    plt.figure("Analyse 1 - Correcteur P (Ki=0, Kd=0)", figsize=(8, 6))
    gains_P = [5, 10, 30, 100]
    
    for kp in gains_P:
        t, y, target = simulate_response(Kp=kp, Ki=0, Kd=0)
        plt.plot(t, y, label=f'Kp={kp}')
        if kp == gains_P[0]: 
            plt.axhline(y=target, color='k', linestyle='--', label='Consigne')

    plt.title("Influence du Gain Proportionnel (P)")
    plt.xlabel("Temps (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True)
    
    # Influence du Gain Intégral (PI)
    plt.figure("Analyse 2 - Correcteur PI (Kp fixé, Kd=0)", figsize=(8, 6))
    kp_fixe = 30 # on décide de garder Kp = 30
    gains_I = [0, 0.5, 10, 30, 50]
    
    for ki in gains_I:
        t, y, target = simulate_response(Kp=kp_fixe, Ki=ki, Kd=0)
        plt.plot(t, y, label=f'Ki={ki}')
        if ki == gains_I[0]:
            plt.axhline(y=target, color='k', linestyle='--', label='Consigne')

    plt.title(f"Influence du Gain Intégral (PI) avec Kp={kp_fixe}")
    plt.xlabel("Temps (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True)
    
    # Influence du Gain Dérivé (PID)
    plt.figure("Analyse 3 - Correcteur PID (Kp, Ki fixés)", figsize=(8, 6))
    kp_fixe = 30
    ki_fixe = 0.5  # on décide de garder le gain intégral le plus faible mais pas nul
    gains_D = [0, 1, 2.5, 5, 10]
    
    for kd in gains_D:
        t, y, target = simulate_response(Kp=kp_fixe, Ki=ki_fixe, Kd=kd)
        plt.plot(t, y, label=f'Kd={kd}')
        if kd == gains_D[0]:
            plt.axhline(y=target, color='k', linestyle='--', label='Consigne')

    plt.title(f"Influence du Gain Dérivé (PID) avec Kp={kp_fixe}, Ki={ki_fixe}")
    plt.xlabel("Temps (s)")
    plt.ylabel("Angle (rad)")
    plt.legend()
    plt.grid(True)
    
    print("Graphiques générés. Fermez les fenêtres pour passer à la simulation interactive.")
    plt.show()



def demo_robot_1R():
    analyze_controllers()
    
    dt = 0.0005
    monde = Univers(name="Robot 1R", dimensions=(4, 3), 
                    gameDimensions=(1200, 800), fps=60, step=dt)
    
    # robot
    pivot_pos = V3D(2, 1.5)
    
    # Pivot est fixe = bodyA (la base)
    pivot = Particule(p0=pivot_pos, fix=True, color='black', name="Pivot")
    
    L_bras = 1.0
    
    # Le Bras est mobile = bodyB
    bras = Barre2D(mass=0.2, long=L_bras, theta=-pi/2, pos=pivot_pos + V3D(0, -L_bras/2), 
                color='orange', nom="Bras")
    
    # Liaison mécanique (le pivot physique)
    liaison_eca = LiaisonPivot(pivot, bras, anchor1=-1, k=20000, c=150)
    
    # Moteur et PID 
    moteur = MoteurCC(R=2.0, ke=0.5, kc=0.5, name="Moteur Axe 1") 
    pid = ControlPIDRobot(Kp=30.0, Ki=0.50, Kd=5.0, output_limit=24.0)
    
    # liaison avec bodyA = pivot, bodyB = bras. 
    liaison_mot = LiaisonMotorisee(pivot, bras, moteur, pid, dt, anchorA=0, anchorB=-1)
    
    cible = Particule(p0=V3D(3, 1.5), fix=True, color='green', name="Cible")
    
    monde.addParticule(pivot, bras, cible)
    
    # On ajoute la liaison motorisée aux générateurs de force
    monde.addGenerators(Gravity(V3D(0, -9.81)), liaison_eca, liaison_mot)
    
    history = {'t': [], 'theta': [], 'target': [], 'u': []}
    
    def myInteraction(self_univ, events, keys):
        # déplacement Cible
        vitesse_cible = 0.05
        if keys[pygame.K_LEFT]:  cible.position[-1].x -= vitesse_cible
        if keys[pygame.K_RIGHT]: cible.position[-1].x += vitesse_cible
        if keys[pygame.K_UP]:    cible.position[-1].y += vitesse_cible
        if keys[pygame.K_DOWN]:  cible.position[-1].y -= vitesse_cible
        
        # calcul Consigne
        diff = cible.getPosition() - pivot.getPosition()
        target_angle = atan2(diff.y, diff.x)
        
        # Mise à jour de la consigne dans le PID de la liaison
        liaison_mot.pid.target = target_angle
        
        # enregistrement
        history['t'].append(self_univ.time[-1])
        
        # On normalise l'angle réel pour qu'il reste entre -pi et pi pour l'affichage
        theta_normalized = (bras.theta + pi) % (2 * pi) - pi
        
        history['theta'].append(theta_normalized)
        history['target'].append(target_angle)
        
        # On récupère la tension calculée par la liaison motorisée
        history['u'].append(liaison_mot.voltage)
    
    monde.gameInteraction = MethodType(myInteraction, monde)
    
    print("\nROBOT 1R avec Liaison Motorisée\n")
    print(" Commandes :")
    print("   [FLECHE HAUT/BAS/GAUCHE/DROITE] : Bouger la cible\n\n")
    
    monde.game = True
    monde.simulateRealTime()
    
    plt.figure("Analyse Asservissement", figsize=(10, 5))
    
    plt.plot(history['t'], history['target'], 'g--', label='Consigne')
    plt.plot(history['t'], history['theta'], 'r-', label='Angle Réel', linewidth=2)
    
    plt.title("Suivi de Position Angulaire")
    plt.ylabel("Angle (rad)")
    plt.xlabel("Temps (s)")
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    demo_robot_1R()