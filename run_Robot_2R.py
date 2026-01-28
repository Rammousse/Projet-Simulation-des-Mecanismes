import pygame
from math import pi, sin, cos
from types import MethodType
import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, LiaisonPivot, LiaisonMotorisee
from particule import Particule
from barre2D import Barre2D
from moteurCC import MoteurCC
from control_PID_Robot import ControlPIDRobot

def demo_robot_2R():
    dt = 0.0001  
    monde = Univers(name="Robot 2R", dimensions=(6, 4), 
                    gameDimensions=(1200, 800), fps=120, step=dt)
    
    pivot_pos = V3D(3, 2.5) 
    L1 = 1.2
    L2 = 1.0
    
    # Création des Corps
    socle = Particule(p0=pivot_pos, fix=True, color='black', name="Socle")
    
    # Bras 1 (Orange)
    bras1 = Barre2D(mass=2.0, long=L1, theta=-pi/2, pos=pivot_pos + V3D(0, -L1/2), 
                    color='orange', nom="Bras 1")
    
    # Bras 2 (Bleu)
    pos_depart_bras2 = pivot_pos + V3D(0, -L1)
    pos_c2 = pos_depart_bras2 + V3D(0, -L2/2)
    bras2 = Barre2D(mass=0.5, long=L2, theta=-pi/2, pos=pos_c2, 
                    color='cyan', nom="Bras 2")
    
    # liaisons Mécaniques
    liaison_eca1 = LiaisonPivot(socle, bras1, anchor1=-1, k=40000, c=500)
    liaison_eca2 = LiaisonPivot(bras1, bras2, anchor0=1, anchor1=-1, k=40000, c=300)
    
    # Motorisation
    moteur1 = MoteurCC(R=0.2, ke=2.0, kc=2.0, name="Moteur 1")
    pid1 = ControlPIDRobot(Kp=800.0, Ki=5.0, Kd=60.0, output_limit=12.0)
    joint1 = LiaisonMotorisee(socle, bras1, moteur1, pid1, dt, anchorA=0, anchorB=-1)
    
    moteur2 = MoteurCC(R=1.0, ke=1.0, kc=1.0, name="Moteur 2")
    pid2 = ControlPIDRobot(Kp=100.0, Ki=2.0, Kd=10.0)
    joint2 = LiaisonMotorisee(bras1, bras2, moteur2, pid2, dt, anchorA=1, anchorB=-1)
    
    # Initialisation
    pid1.target = -pi/2 
    pid2.target = 0.0 
    
    monde.addParticule(socle, bras1, bras2)
    monde.addGenerators(Gravity(V3D(0, -9.81)), liaison_eca1, liaison_eca2, 
                        joint1, joint2)
    
    history = {'t': [], 'th1': [], 'th1_ref': [], 'th2': [], 'th2_ref': []}
    
    def control_articulaire(self_univ, events, keys):
        
        if (keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]):
            step_angle = 0.05
        else: 
            step_angle = 0.01
        
        # Modification de la consigne Moteur 1
        if keys[pygame.K_UP]:   joint1.pid.target += step_angle
        if keys[pygame.K_DOWN]: joint1.pid.target -= step_angle
        
        # Modification de la consigne Moteur 2
        if keys[pygame.K_LEFT]:  joint2.pid.target += step_angle
        if keys[pygame.K_RIGHT]: joint2.pid.target -= step_angle
        
        # On enregistre si le temps a avancé depuis la dernière fois
        if len(self_univ.time) > 0:
            current_t = self_univ.time[-1]
            
            history['t'].append(current_t)
            history['th1'].append(bras1.theta)
            history['th1_ref'].append(joint1.pid.target)
            history['th2'].append(bras2.theta)
            history['th2_ref'].append(joint2.pid.target)
    
    monde.gameInteraction = MethodType(control_articulaire, monde)
    
    print("\n ROBOT 2R\n")
    print("Commandes :")
    print("  [HAUT / BAS]      : Epaule")
    print("  [GAUCHE / DROITE] : Coude")
    print("  [MAINTENIR SHIFT] : Mode Turbo (Vitesse x5)\n")
    
    monde.game = True
    monde.simulateRealTime()
    
    # Affichage final
    if len(history['t']) > 1:
        plt.figure("Analyse Robot 2R", figsize=(10, 8))
        
        # remettre les angles entre -pi et +pi
        def normalise(angles):
            return [(a + pi) % (2 * pi) - pi for a in angles]
        
        # graphique Moteur 1
        plt.subplot(2, 1, 1)
        plt.plot(history['t'], normalise(history['th1_ref']), 'g--', label='Consigne 1')
        plt.plot(history['t'], normalise(history['th1']), 'r-', label='Mesure 1')
        plt.ylabel("Angle 1 (rad)")
        plt.legend()
        plt.grid(True)
        
        # Graphique Moteur 2
        # Calcul de l'angle relatif
        th2_rel = [(h2 - h1 + pi) % (2*pi) - pi for h1, h2 in zip(history['th1'], history['th2'])]
        
        plt.subplot(2, 1, 2)
        plt.plot(history['t'], normalise(history['th2_ref']), 'g--', label='Consigne 2')
        plt.plot(history['t'], th2_rel, 'b-', label='Mesure 2')
        plt.ylabel("Angle 2 (rad)")
        plt.legend()
        plt.grid(True)
        
        plt.show()

if __name__ == '__main__':
    demo_robot_2R()