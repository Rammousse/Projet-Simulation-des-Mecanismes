import pygame
import numpy as np
from math import pi, sin, cos, sqrt, atan2
from types import MethodType
import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, LiaisonPivot, LiaisonMotorisee
from particule import Particule
from barre2D import Barre2D
from moteurCC import MoteurCC
from control_PID_Robot import ControlPIDRobot

def MGD(q1, q2, L1, L2, origin_pos):
    """
    Calcule la position cartésienne (X, Y) de l'effecteur
    basée sur les angles actuels.
    """
    x = origin_pos.x + L1 * cos(q1) + L2 * cos(q1 + q2)
    y = origin_pos.y + L1 * sin(q1) + L2 * sin(q1 + q2)
    return V3D(x, y)

def jacobienne(q1, q2, L1, L2):
    """
    Retourne la matrice Jacobienne 2x2 du robot 2R.
    """
    theta1 = q1
    theta12 = q1 + q2
    s1 = sin(theta1)
    c1 = cos(theta1)
    s12 = sin(theta12)
    c12 = cos(theta12)
    
    j11 = -L1 * s1 - L2 * s12
    j12 = -L2 * s12
    j21 = L1 * c1 + L2 * c12
    j22 = L2 * c12
    
    return np.array([[j11, j12], [j21, j22]])

def demo_robot_2R_MCI():
    dt = 0.0005
    monde = Univers(name="Robot 2R - MCI Control", dimensions=(6, 4), 
                    gameDimensions=(1200, 800), fps=60, step=dt)
    
    pivot_pos = V3D(3, 2.5) 
    L1 = 1.2
    L2 = 1.0
    
    # création du Robot
    socle = Particule(p0=pivot_pos, fix=True, color='black', name="Socle")
    
    init_q1 = -pi/2
    init_q2 = 0.0
    
    bras1 = Barre2D(mass=2.0, long=L1, theta=init_q1, 
                    pos=pivot_pos + V3D(0, -L1/2), 
                    color='orange', nom="Bras 1")
    
    pos_depart_bras2 = pivot_pos + V3D(0, -L1)
    pos_c2 = pos_depart_bras2 + V3D(0, -L2/2)
    bras2 = Barre2D(mass=0.5, long=L2, theta=init_q1 + init_q2, 
                    pos=pos_c2, color='cyan', nom="Bras 2")
    
    cible_visuelle = Particule(p0=pos_depart_bras2 + V3D(0, -L2), fix=True, 
                            color='green', name="Cible")
    
    liaison_eca1 = LiaisonPivot(socle, bras1, anchor1=-1, k=40000, c=500)
    liaison_eca2 = LiaisonPivot(bras1, bras2, anchor0=1, anchor1=-1, k=40000, c=300)
    
    # motorisation et contrôle
    moteur1 = MoteurCC(R=0.5, ke=2.0, kc=2.0, name="Moteur 1")
    pid1 = ControlPIDRobot(Kp=1500.0, Ki=2.0, Kd=200.0, output_limit=24.0)
    joint1 = LiaisonMotorisee(socle, bras1, moteur1, pid1, dt, anchorA=0, anchorB=-1)
    
    moteur2 = MoteurCC(R=1.0, ke=1.0, kc=1.0, name="Moteur 2")
    pid2 = ControlPIDRobot(Kp=800.0, Ki=2.0, Kd=80.0, output_limit=12.0)
    joint2 = LiaisonMotorisee(bras1, bras2, moteur2, pid2, dt, anchorA=1, anchorB=-1)
    
    pid1.target = init_q1
    pid2.target = init_q2
    
    monde.addParticule(socle, bras1, bras2, cible_visuelle)
    monde.addGenerators(Gravity(V3D(0, -9.81)), liaison_eca1, liaison_eca2, joint1, joint2)
    
    # Position cible active
    target_cartesian = MGD(init_q1, init_q2, L1, L2, pivot_pos)
    
    # Position de départ du mouvement (pour calculer l'écart à la ligne)
    start_cartesian = V3D(target_cartesian.x, target_cartesian.y)
    
    V_max = 6.0           
    dist_threshold = 0.001 
    braking_distance = 0.2 
    
    history = {'t': [], 'dist_err': [], 'lateral_err': [], 'q1': [], 'q2': []}
    
    def control_MCI(self_univ, events, keys):
        nonlocal target_cartesian, start_cartesian
        
        # gestion de la Souris
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                x_mouse, y_mouse = pygame.mouse.get_pos()
                real_x = x_mouse / self_univ.scale
                real_y = (self_univ.gameDimensions[1] - y_mouse) / self_univ.scale
                
                # avant de changer la cible, on enregistre où on est (pour définir la ligne droite)
                # On utilise la position actuelle de l'effecteur comme départ
                curr_q1 = bras1.theta
                curr_q2 = bras2.theta - bras1.theta
                start_cartesian = MGD(curr_q1, curr_q2, L1, L2, pivot_pos)
                
                target_cartesian = V3D(real_x, real_y)
                cible_visuelle.position[-1] = target_cartesian
                print(f"Cible : {target_cartesian}")
        
        # récupération de l'etat actel
        current_q1 = bras1.theta
        current_q2 = bras2.theta - bras1.theta
        current_effector_pos = MGD(current_q1, current_q2, L1, L2, pivot_pos)
        
        # erreur de distance
        diff_vec = target_cartesian - current_effector_pos
        dist_error = diff_vec.mod()
        
        # erreur latérale Ecart à la ligne
        vec_traj_x = target_cartesian.x - start_cartesian.x
        vec_traj_y = target_cartesian.y - start_cartesian.y
        len_traj = sqrt(vec_traj_x**2 + vec_traj_y**2)
        
        lateral_error = 0.0
        if len_traj > 0.0001:
            # Vecteur AP (Départ -> Position Actuelle)
            vec_curr_x = current_effector_pos.x - start_cartesian.x
            vec_curr_y = current_effector_pos.y - start_cartesian.y
            
            # Produit vectoriel pour la distance perpendiculaire
            cross_prod = vec_traj_x * vec_curr_y - vec_traj_y * vec_curr_x
            lateral_error = abs(cross_prod) / len_traj
        
        # génération de la Vitesse
        dX = np.zeros((2, 1))
        
        if dist_error > dist_threshold:
            direction = diff_vec.norm()
            if dist_error < braking_distance:
                speed_factor = dist_error / braking_distance
                current_speed = max(0.1, V_max * speed_factor)
            else:
                current_speed = V_max
            
            v_cmd = direction * current_speed
            dX[0, 0] = v_cmd.x
            dX[1, 0] = v_cmd.y
        else:
            dX[0, 0] = 0.0
            dX[1, 0] = 0.0
        
        J = jacobienne(current_q1, current_q2, L1, L2)
        J_pinv = np.linalg.pinv(J)
        dq = np.dot(J_pinv, dX)
        
        dq1 = dq[0, 0]
        dq2 = dq[1, 0]
        
        limit_w = 20.0 
        dq1 = max(-limit_w, min(limit_w, dq1))
        dq2 = max(-limit_w, min(limit_w, dq2))
        
        # intégration
        joint1.pid.target += dq1 * dt
        joint2.pid.target += dq2 * dt
        
        # enregistrement
        if len(self_univ.time) > 0:
            history['t'].append(self_univ.time[-1])
            history['dist_err'].append(dist_error)
            history['lateral_err'].append(lateral_error)
            history['q1'].append(current_q1)
            history['q2'].append(current_q2)
    
    monde.gameInteraction = MethodType(control_MCI, monde)
    
    print("\n ROBOT 2R - COMMANDE MCI (Suivi de Trajectoire) :\n")
    print(" Commandes :")
    print("   [CLIC GAUCHE] : Définir la cible pour l'effecteur")
    
    monde.game = True
    monde.simulateRealTime()
    
    if len(history['t']) > 1:
        plt.figure("Analyse Erreur de Trajectoire", figsize=(10, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(history['t'], history['dist_err'], 'b-', label='Distance à la cible')
        plt.title("Erreur Longitudinale (Distance restant à parcourir)")
        plt.ylabel("Distance (m)")
        plt.grid(True)
        plt.legend()
        
        plt.subplot(2, 1, 2)
        plt.plot(history['t'], history['lateral_err'], 'r-', label='Écart à la ligne droite')
        plt.title("Erreur Latérale (Déviation par rapport à la trajectoire idéale)")
        plt.xlabel("Temps (s)")
        plt.ylabel("Erreur (m)")
        
        #on force l'échelle Y proche de 0 si l'erreur est minime
        max_lat = max(history['lateral_err'])
        if max_lat < 0.05: 
            plt.ylim(-0.001, 0.05)
            
        plt.grid(True)
        plt.legend()
        
        plt.tight_layout()
        plt.show()

if __name__ == '__main__':
    demo_robot_2R_MCI()