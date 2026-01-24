import pygame
from math import pi, sin, cos
from univers import Univers, Gravity, SpringDamper, LiaisonPivot
from particule import Particule
from barre2D import Barre2D
from vector3D import Vector3D as V3D

def demo_pendule_vs_barre():
    monde = Univers(name="Comparaison Pendules", dimensions=(30, 20), 
                    gameDimensions=(1200, 800), fps=60, step=0.001)
    
    # Paramètres de simulation
    L_ref = 5.0
    Y_pivot = 15.0 
    theta_0 = pi / 4 
    
    # Pendule Simple de longueur L
    x1 = 5.0
    pivot1 = Particule(p0=V3D(x1, Y_pivot), fix=True, color='black', name="Pivot1")
    
    pos_m1 = pivot1.getPosition() + V3D(L_ref * sin(theta_0), -L_ref * cos(theta_0))
    masse1 = Particule(p0=pos_m1, mass=1, color='red', name="Pendule Simple (L)")
    
    tige1 = SpringDamper(pivot1, masse1, k=5000, c=100, l0=L_ref)
    
    # Barre de longueur 2L
    x2 = 15.0
    L_bar2 = 2.0 * L_ref
    pivot2 = Particule(p0=V3D(x2, Y_pivot), fix=True, color='black', name="Pivot2")
    
    u_bar2 = V3D(sin(theta_0), -cos(theta_0))
    pos_c2 = pivot2.getPosition() + u_bar2 * (L_bar2 / 2.0)
    
    angle_barre = -pi/2 + theta_0
    
    barre2 = Barre2D(mass=1, long=L_bar2, theta=angle_barre, pos=pos_c2, 
                    color='blue', nom="Barre (2L)")
    
    liaison2 = LiaisonPivot(pivot2, barre2, anchor1=-1, k=5000, c=100)
    
    
    # Barre de longueur 1.5L
    x3 = 25.0
    L_bar3 = 1.5 * L_ref
    pivot3 = Particule(p0=V3D(x3, Y_pivot), fix=True, color='black', name="Pivot3")
    
    u_bar3 = V3D(sin(theta_0), -cos(theta_0))
    pos_c3 = pivot3.getPosition() + u_bar3 * (L_bar3 / 2.0)
    
    barre3 = Barre2D(mass=1, long=L_bar3, theta=angle_barre, pos=pos_c3, 
                    color='green', nom="Barre (1.5L)")
    
    liaison3 = LiaisonPivot(pivot3, barre3, anchor1=-1, k=5000, c=100)
    
    monde.addParticule(pivot1, masse1)
    monde.addParticule(pivot2, barre2)
    monde.addParticule(pivot3, barre3)
    
    g = Gravity(V3D(0, -9.81))
    monde.addGenerators(g, tige1, liaison2, liaison3)
    
    print("Simulation Pendule vs Barre 2D :")
    print(f"Pendule Simple : L={L_ref}m (Rouge)")
    print(f"Barre Demandée : L={L_bar2}m (Bleu)")
    print(f"Barre Corrigée : L={L_bar3}m (Vert)")
    
    monde.game = True
    monde.simulateRealTime()


if __name__ == '__main__':
    demo_pendule_vs_barre()