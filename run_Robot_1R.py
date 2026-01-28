import pygame
from math import pi, atan2
from types import MethodType
import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, LiaisonPivot, LiaisonMotorisee
from particule import Particule
from barre2D import Barre2D
from moteurCC import MoteurCC
from control_PID_Robot import ControlPIDRobot1R


def demo_robot_1R():
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
    pid = ControlPIDRobot1R(Kp=30.0, Ki=1.0, Kd=2.5, output_limit=12.0)
    
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