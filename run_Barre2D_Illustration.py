import pygame
from math import pi, cos, sin
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers, Gravity, SpringDamper, Viscosity, LiaisonPivot, BoxBoundaries
from particule import Particule
from barre2D import Barre2D

# on rempalce la méthode .gameDraw() par défaut juste pour la tête
def draw_grosse_tete(self, scale, screen):
    # Calcul de la position écran
    X = int(scale * self.getPosition().x)
    Y = int(scale * self.getPosition().y)
    
    # Rayon de la tête
    RAYON = 20
    
    # Gestion de la couleur (string ou tuple)
    if isinstance(self.color, str):
        c = pygame.Color(self.color)
    else:
        c = self.color
    
    # Dessin du disque
    pygame.draw.circle(screen, c, (X, Y), RAYON)

def build_pantin(univers, pos_center):
    
    # Tronc 
    tronc = Barre2D(mass=5, long=2.0, theta=pi/2, pos=pos_center, 
                    color="red", nom="Tronc")
    
    pos_haut_tronc = pos_center + V3D(0, 1.0)
    pos_bas_tronc  = pos_center + V3D(0, -1.0)
    
    # Tête
    head = Particule(mass=1.5, p0=pos_haut_tronc + V3D(0, 0.4), v0=V3D(), 
                    color="red", name="Tete")
    
    # On remplace la méthode gameDraw par la grosse tete
    head.gameDraw = MethodType(draw_grosse_tete, head)
    
    # Bras
    pos_bras_g = pos_haut_tronc + V3D(-0.75, 0)
    bras_g = Barre2D(mass=1, long=1.5, theta=0, pos=pos_bras_g, 
                    color="red", nom="BrasG")
    
    pos_bras_d = pos_haut_tronc + V3D(0.75, 0)
    bras_d = Barre2D(mass=1, long=1.5, theta=0, pos=pos_bras_d, 
                    color="red", nom="BrasD")
    
    # Jambes
    pos_jambe_g = pos_bas_tronc + V3D(-0.3, -1.0)
    jambe_g = Barre2D(mass=2, long=2.0, theta=-pi/2, pos=pos_jambe_g, 
                    color="red", nom="JambeG")
    
    pos_jambe_d = pos_bas_tronc + V3D(0.3, -1.0)
    jambe_d = Barre2D(mass=2, long=2.0, theta=-pi/2, pos=pos_jambe_d, 
                    color="red", nom="JambeD")
    
    # Assemblage
    AMORTISSEMENT_ARTICULATION = 300 
    RAIDEUR = 2000
    
    cou = SpringDamper(tronc, head, k=1000, c=AMORTISSEMENT_ARTICULATION, l0=0.4, anchor0=1)
    epaule_g = LiaisonPivot(tronc, bras_g, anchor0=1, anchor1=1, k=RAIDEUR, c=AMORTISSEMENT_ARTICULATION)
    epaule_d = LiaisonPivot(tronc, bras_d, anchor0=1, anchor1=-1, k=RAIDEUR, c=AMORTISSEMENT_ARTICULATION)
    anche_g = LiaisonPivot(tronc, jambe_g, anchor0=-1, anchor1=-1, k=RAIDEUR, c=AMORTISSEMENT_ARTICULATION)
    anche_d = LiaisonPivot(tronc, jambe_d, anchor0=-1, anchor1=-1, k=RAIDEUR, c=AMORTISSEMENT_ARTICULATION)
    
    univers.addParticule(head, tronc, bras_g, bras_d, jambe_g, jambe_d)
    univers.addGenerators(cou, epaule_g, epaule_d, anche_g, anche_d)
    
    return head, tronc


def demo_pendu():
    
    monde = Univers(name="Pantin", dimensions=(30, 25), 
                    gameDimensions=(1024, 800), fps=60, step=0.001)
    
    # Pivot Fixe
    pos_pivot = V3D(15, 22)
    pivot = Particule(p0=pos_pivot, fix=True, name="Pivot", color=(0,0,0))
    monde.addParticule(pivot)
    
    # Création du pantin
    pos_depart_pantin = V3D(15, 15)
    tete_pantin, tronc_pantin = build_pantin(monde, pos_depart_pantin)
    
    # Corde
    longueur_corde = 7.0
    corde = SpringDamper(pivot, tete_pantin, k=10000, c=50, l0=longueur_corde, name="Corde")
    monde.addGenerators(corde)
    
    # Environnement
    g = Gravity(V3D(0, -9.81))
    box = BoxBoundaries(0, 30, 0, 25)
    air = Viscosity(coeff=0.5)
    monde.addGenerators(g, box, air)
    
    # Variables de contrôle
    monde.oscillation_active = False
    monde.freq = 1.0  # rad/s
    monde.amp = 50.0  # Force
    monde.corde_ref = corde 
    
    def custom_interaction(self, events, keys):
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.oscillation_active = not self.oscillation_active
                    print(f"Oscillation : {self.oscillation_active}")
                
                if event.key == pygame.K_UP:
                    self.freq += 0.1
                    print(f"Fréquence : {self.freq:.1f} rad/s")
                if event.key == pygame.K_DOWN:
                    self.freq -= 0.1
                    if self.freq < 0: self.freq = 0
                    print(f"Fréquence : {self.freq:.1f} rad/s")
                
                if event.key == pygame.K_LEFT:
                    self.corde_ref.l0 -= 0.5
                    if self.corde_ref.l0 < 1: self.corde_ref.l0 = 1
                    print(f"Longueur corde : {self.corde_ref.l0:.1f} m")
                if event.key == pygame.K_RIGHT:
                    self.corde_ref.l0 += 0.5
                    print(f"Longueur corde : {self.corde_ref.l0:.1f} m")
        
        # Application de la force
        if self.oscillation_active:
            t = self.time[-1]
            force_val = self.amp * cos(self.freq * t)
            tronc_pantin.applyEffort(Force=V3D(force_val, 0, 0), Point=0)
    
    monde.gameInteraction = MethodType(custom_interaction, monde)
    
    print("Commandes :")
    print(" [ESPACE] : Activer/Désactiver le moteur d'excitation")
    print(" [HAUT] : Augmenter la fréquence (+0.1 rad/s)")
    print(" [BAS] : Diminuer la fréquence (-0.1 rad/s)")
    print(" [GAUCHE] : Diminuer la taille de la corde")
    print(" [DROITE] : Augmenter la taille de la corde")
    
    monde.game = True
    monde.simulateRealTime()

if __name__ == '__main__':
    demo_pendu()