import pygame
from math import pi, sin, cos, acos, atan2, sqrt
from types import MethodType
import matplotlib.pyplot as plt
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, LiaisonPivot, LiaisonMotorisee
from particule import Particule
from barre2D import Barre2D
from moteurCC import MoteurCC
from control_PID_Robot import ControlPIDRobot

def MGI(target_pos, origin_pos, L1, L2):
    """
    Calcule le Modèle Géométrique Inverse pour un robot 2R.
    Retourne une liste de tuples [(theta1_a, theta2_a), (theta1_b, theta2_b)].
    """
    # Vecteur de l'origine vers la cible
    dx = target_pos.x - origin_pos.x
    dy = target_pos.y - origin_pos.y
    dist_sq = dx**2 + dy**2
    dist = sqrt(dist_sq)
    
    # Vérification si la cible est atteignabile
    if dist > (L1 + L2):
        scale = (L1 + L2) / dist
        dx *= scale
        dy *= scale
        dist = L1 + L2
        dist_sq = dist**2
    
    # Si la cible est trop proche  on s'arrête à la limite
    if dist < abs(L1 - L2):
        return [] # Pas de solution stable ou ignorer
    
    # Calcul de l'angle du coude theta2
    # dist^2 = L1^2 + L2^2 - 2*L1*L2*cos(pi - theta2)
    # cos(theta2) = (dist^2 - L1^2 - L2^2) / (2*L1*L2)
    c2 = (dist_sq - L1**2 - L2**2) / (2 * L1 * L2)
    
    # pour éviter les erreurs numériques d'arrondi
    c2 = max(-1.0, min(1.0, c2))
    
    # Deux solutions pour theta2 coude "bas" et Coude "haut"
    th2_a = -acos(c2)
    th2_b = acos(c2)
    
    solutions = []
    
    # calcul de theta1 pour chaque theta2
    for th2 in [th2_a, th2_b]:
        # Angle de la configuration géométrique
        k1 = L1 + L2 * cos(th2)
        k2 = L2 * sin(th2)
        
        # theta1 = angle du vecteur cible
        th1 = atan2(dy, dx) - atan2(k2, k1)
        
        solutions.append((th1, th2))
        
    return solutions

def best_solution(solutions, current_th1, current_th2):
    """
    Choisit la solution la plus proche de la configuration actuelle.
    """
    best_sol = None
    min_diff = float('inf')
    
    def normalize_angle(a):
        return (a + pi) % (2 * pi) - pi
    
    for (th1_ref, th2_ref) in solutions:
        # Calcul de la distance dans l'espace articulaire
        diff1 = normalize_angle(th1_ref - current_th1)
        diff2 = normalize_angle(th2_ref - current_th2)
        
        # Norme au carré de l'erreur
        total_diff = diff1**2 + diff2**2
        
        if total_diff < min_diff:
            min_diff = total_diff
            best_sol = (th1_ref, th2_ref)
            
    return best_sol

def demo_robot_2R_MGI():
    dt = 0.0001  
    monde = Univers(name="Robot 2R - Inverse Kinematics", dimensions=(6, 4), 
                    gameDimensions=(1200, 800), fps=120, step=dt)
    
    pivot_pos = V3D(3, 2.5) 
    L1 = 1.2
    L2 = 1.0
    
    # Création des Corps
    socle = Particule(p0=pivot_pos, fix=True, color='black', name="Socle")
    
    bras1 = Barre2D(mass=2.0, long=L1, theta=-pi/2, pos=pivot_pos + V3D(0, -L1/2), 
                    color='orange', nom="Bras 1")
    
    pos_depart_bras2 = pivot_pos + V3D(0, -L1)
    pos_c2 = pos_depart_bras2 + V3D(0, -L2/2)
    bras2 = Barre2D(mass=0.5, long=L2, theta=-pi/2, pos=pos_c2, 
                    color='cyan', nom="Bras 2")
    
    # Cible visuelle (point vert)
    cible_visuelle = Particule(p0=pos_depart_bras2 + V3D(0, -L2), fix=True, color='green', name="Cible")
    
    # Liaisons Mécaniques
    liaison_eca1 = LiaisonPivot(socle, bras1, anchor1=-1, k=40000, c=500)
    liaison_eca2 = LiaisonPivot(bras1, bras2, anchor0=1, anchor1=-1, k=40000, c=300)
    
    #gains changés par rapport au robot 2R manuel pour éviter le dépassement
    # Moteur 1 (Epaule)
    moteur1 = MoteurCC(R=0.5, ke=2.0, kc=2.0, name="Moteur 1")
    pid1 = ControlPIDRobot(Kp=1200.0, Ki=0.1, Kd=250.0, output_limit=24.0)
    joint1 = LiaisonMotorisee(socle, bras1, moteur1, pid1, dt, anchorA=0, anchorB=-1)
    
    # Moteur 2 (Coude)
    moteur2 = MoteurCC(R=1.0, ke=1.0, kc=1.0, name="Moteur 2")
    pid2 = ControlPIDRobot(Kp=600.0, Ki=0.1, Kd=100.0, output_limit=12.0)
    joint2 = LiaisonMotorisee(bras1, bras2, moteur2, pid2, dt, anchorA=1, anchorB=-1)
    
    # Initialisation des consignes
    pid1.target = -pi/2 
    pid2.target = 0.0 
    
    monde.addParticule(socle, bras1, bras2, cible_visuelle)
    monde.addGenerators(Gravity(V3D(0, -9.81)), liaison_eca1, liaison_eca2, joint1, joint2)
    
    history = {'t': [], 'th1': [], 'th1_ref': [], 'th2': [], 'th2_ref': []}
    
    def mouse_control_MGI(self_univ, events, keys):
        
        # Détection du clic souris
        mouse_clicked = False
        target_coords = None
        
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                x_mouse, y_mouse = pygame.mouse.get_pos()
                
                # Conversion Pixel
                real_x = x_mouse / self_univ.scale
                real_y = (self_univ.gameDimensions[1] - y_mouse) / self_univ.scale
                target_coords = V3D(real_x, real_y)
                mouse_clicked = True
        
        # si clic on calcule et on met à jour
        if mouse_clicked and target_coords:
            
            # maj visuelle
            cible_visuelle.position[-1] = target_coords
            
            # calcul des solutions inverses
            solutions = MGI(target_coords, pivot_pos, L1, L2)
            
            if solutions:
                # récupération des angles actuel
                current_th1 = bras1.theta
                current_th2 = bras2.theta - bras1.theta
                
                # sélection de la solution la plus proche
                best = best_solution(solutions, current_th1, current_th2)
                
                if best:
                    th1_cmd, th2_cmd = best
                    
                    # maj des consignes PID
                    joint1.pid.target = th1_cmd
                    joint2.pid.target = th2_cmd
                    
                    print(f"Cible : {target_coords}")
                    print(f"  Solution : Th1={th1_cmd:.2f}, Th2={th2_cmd:.2f}")
                else:
                    print("Erreur selection solution")
            else:
                print("Cible hors de portée !")
        
        # Enregistrement pour graphiques
        if len(self_univ.time) > 0:
            current_t = self_univ.time[-1]
            history['t'].append(current_t)
            history['th1'].append(bras1.theta)
            history['th1_ref'].append(joint1.pid.target)
            history['th2'].append(bras2.theta)
            
            # L'angle réel du moteur 2 est relatif
            history['th2_ref'].append(joint2.pid.target)
    
    monde.gameInteraction = MethodType(mouse_control_MGI, monde)
    
    print("\n ROBOT 2R - COMMANDE SOURIS MGI :\n")
    print("Commandes :")
    print("  [CLIC GAUCHE] : Définir la cible pour l'effecteur")
    print("  Le robot choisira automatiquement la configuration la plus proche.\n")
    
    monde.game = True
    monde.simulateRealTime()
    
    if len(history['t']) > 1:
        plt.figure("Analyse Robot 2R - MGI", figsize=(10, 8))
        
        def normalise(angles):
            return [(a + pi) % (2 * pi) - pi for a in angles]
        
        # Moteur 1
        plt.subplot(2, 1, 1)
        plt.plot(history['t'], normalise(history['th1_ref']), 'g--', label='Consigne 1')
        plt.plot(history['t'], normalise(history['th1']), 'r-', label='Mesure 1')
        plt.ylabel("Angle 1 (rad)")
        plt.title("Réponse à la commande Souris")
        plt.legend()
        plt.grid(True)
        
        # Moteur 2
        # Calcul de l'angle relatif mesuré pour comparer à la consigne relative
        th2_rel = [(h2 - h1 + pi) % (2*pi) - pi for h1, h2 in zip(history['th1'], history['th2'])]
        
        plt.subplot(2, 1, 2)
        plt.plot(history['t'], normalise(history['th2_ref']), 'g--', label='Consigne 2 (Relatif)')
        plt.plot(history['t'], th2_rel, 'b-', label='Mesure 2 (Relatif)')
        plt.ylabel("Angle 2 (rad)")
        plt.xlabel("Temps (s)")
        plt.legend()
        plt.grid(True)
        
        plt.show()

if __name__ == '__main__':
    demo_robot_2R_MGI()