import pygame
from math import pi, sin, cos
from univers import Univers, Gravity, SpringDamper, LiaisonPivot
from particule import Particule
from barre2D import Barre2D
from vector3D import Vector3D as V3D
from types import MethodType

def forced_interaction(self, events, keys):
    for event in events:
        if event.type == pygame.KEYDOWN:
            # Gestion Fréquence
            if event.key == pygame.K_UP:
                self.forcing_freq += 0.1
                print(f"Fréquence d'excitation : {self.forcing_freq:.2f} rad/s")
            if event.key == pygame.K_DOWN:
                self.forcing_freq -= 0.1
                if self.forcing_freq < 0: self.forcing_freq = 0
                print(f"Fréquence d'excitation : {self.forcing_freq:.2f} rad/s")
            
            # Gestion Activation
            if event.key == pygame.K_SPACE:
                self.forcing_active = not self.forcing_active
                etat = "ACTIVÉE" if self.forcing_active else "STOPPÉE"
                print(f"Excitation {etat}")
    
    # aplication de la force sinusoïdale
    if self.forcing_active:
        t = self.time[-1]
        force_val = self.forcing_amp * cos(self.forcing_freq * t)
        F_vec = V3D(force_val, 0, 0)
        
        self.barre_cible.applyEffort(Force=F_vec, Point=1)


def demo_pendules_couples():
    """
    Simulation de deux barres couplées avec excitation forcée.
    """
    monde = Univers(name="Pendules Couplés (Clean)", dimensions=(30, 20), 
                    gameDimensions=(1200, 800), fps=60, step=0.001)
    
    L_barre = 6.0
    Y_pivot = 15.0
    X_pivot1 = 10.0
    Ecart = 10.0
    X_pivot2 = X_pivot1 + Ecart
    
    # Choix du mode pour la démonstration :
    # 0: Repos (pour activer le forçage), 1: Symétrique, 2: Antisymétrique
    mode = 0
    
    if mode == 0:
        theta1, theta2 = 0.0, 0.0
    elif mode == 1:
        theta1, theta2 = 0.3, 0.3
    elif mode == 2:
        theta1, theta2 = 0.2, -0.2
    
    piv1 = Particule(p0=V3D(X_pivot1, Y_pivot), fix=True, color='black', name="Piv1")
    piv2 = Particule(p0=V3D(X_pivot2, Y_pivot), fix=True, color='black', name="Piv2")
    
    # Barre 1
    u1 = V3D(sin(theta1), -cos(theta1)) 
    pos_c1 = piv1.getPosition() + u1 * (L_barre / 2.0)
    barre1 = Barre2D(mass=1, long=L_barre, theta=(-pi/2 + theta1), pos=pos_c1, 
                    color='red', nom="Pendule 1 (Excité)")
    
    # Barre 2
    u2 = V3D(sin(theta2), -cos(theta2))
    pos_c2 = piv2.getPosition() + u2 * (L_barre / 2.0)
    barre2 = Barre2D(mass=1, long=L_barre, theta=(-pi/2 + theta2), pos=pos_c2, 
                    color='blue', nom="Pendule 2")
    
    # Liaisons et Ressorts
    liaison1 = LiaisonPivot(piv1, barre1, anchor1=-1, k=10000, c=50)
    liaison2 = LiaisonPivot(piv2, barre2, anchor1=-1, k=10000, c=50)
    ressort_couple = SpringDamper(barre1, barre2, k=30, c=0.05, l0=Ecart, 
                                anchor0=1, anchor1=1, name="Couplage")
    
    monde.addParticule(piv1, piv2)
    monde.addParticule(barre1, barre2)
    
    grav = Gravity(V3D(0, -9.81))
    monde.addGenerators(grav, liaison1, liaison2, ressort_couple)
    
    
    # on stocke les variables nécessaires dans l'univers
    monde.forcing_freq = 1.5
    monde.forcing_amp = 30.0
    monde.forcing_active = False
    monde.barre_cible = barre1
    
    #von injecte la méthode définie avant
    monde.gameInteraction = MethodType(forced_interaction, monde)
    
    print("Commandes :")
    print("  [ESPACE] : Activer/Désactiver le moteur d'excitation")
    print("  [HAUT]   : Augmenter la fréquence (+0.1 rad/s)")
    print("  [BAS]    : Diminuer la fréquence (-0.1 rad/s)")
    print(f" Fréquence initiale : {monde.forcing_freq} rad/s")
    
    monde.game = True
    monde.simulateRealTime()

if __name__ == '__main__':
    demo_pendules_couples()