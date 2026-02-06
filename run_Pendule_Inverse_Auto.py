import pygame
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers, Pichenette
from pendule_Inverse import PenduleInverse
from control_PID_Pendule_Inverse import ControlPID_Pendule

def demo_pendule_inverse_auto():
    
    monUnivers = Univers(name="Pendule Inverse Auto", dimensions=(20, 15), 
                        gameDimensions=(1200, 800), fps=60, step=0.001)
    
    # Instanciation du système physique
    systeme = PenduleInverse(monUnivers)

    # On définit une nouvelle fonction de dessin qui trace que le chariot sans la force
    def draw_sans_vecteur(self, scale, screen):
        X = int(scale * self.getPosition().x)
        Y = int(scale * self.getPosition().y)
        size = 3
        
        if type(self.color) is tuple:
            c = (self.color[0]*255, self.color[1]*255, self.color[2]*255)
        else:
            c = self.color
            
        # On dessine seulement le cercle, pas la ligne
        pygame.draw.circle(screen, c, (X, Y), size*2, size)

    # On remplace la méthode de dessin du chariot par notre version
    systeme.base.gameDraw = MethodType(draw_sans_vecteur, systeme.base)
    
    # Instanciation du controleur
    # On vise la position X=10 (milieu) et l'angle Pi/2 (Verticale)
    controller = ControlPID_Pendule(x_target=10.0, theta_target=3.14159/2)

    def myInteraction(self, events, keys):
        
        # gestion des événements ponctuels (Pichenettes)
        for event in events:
            if event.type == pygame.KEYDOWN:
                
                # Perturbation vers la Droite (Flèche Droite) -> Force sur le sommet du pendule
                if event.key == pygame.K_RIGHT:
                    print(">>> Pichenette Droite !")
                    # On crée une force temporaire via la classe Pichenette de univers.py
                    p = Pichenette(target=systeme.pendule, 
                                   force_vector=V3D(8, 0, 0), # Force en Newton
                                   duration=0.15,              # Durée en secondes
                                   step=self.step)
                    self.addGenerators(p) # Ajout à la simulation
                
                # Perturbation vers la Gauche (Flèche Gauche)
                elif event.key == pygame.K_LEFT:
                    print("<<< Pichenette Gauche !")
                    p = Pichenette(target=systeme.pendule, 
                                   force_vector=V3D(-8, 0, 0), 
                                   duration=0.15, 
                                   step=self.step)
                    self.addGenerators(p)
                    
                # Reset du système
                elif event.key == pygame.K_r:
                    print("Reset Système")
                    systeme.reset()
                    # On nettoie les générateurs de force temporaires s'il y en a
                    self.generators = [g for g in self.generators if not isinstance(g, Pichenette)]

        # Boucle de Contrôle Automatique
        # Récupération de l'état du système
        theta = systeme.pendule.theta
        omega = systeme.pendule.omega
        x_chariot = systeme.base.getPosition().x
        v_chariot = systeme.base.getSpeed().x
        
        # Calcul de la commande via le PID
        force_calculee = controller.compute(theta, omega, x_chariot, v_chariot)
        
        # Application au moteur
        systeme.set_commande(force_calculee)

    # Injection de la méthode d'interaction dans l'univers
    monUnivers.gameInteraction = MethodType(myInteraction, monUnivers)
    
    # Lancement de la simulation
    monUnivers.game = True
    monUnivers.simulateRealTime()

if __name__ == '__main__':
    print("\n PENDULE INVERSE AUTOMATIQUE \n")
    print(" Contrôles :")
    print("  [Flèche Droite] : Pousser le sommet à Droite (Pichenette)")
    print("  [Flèche Gauche] : Pousser le sommet à Gauche (Pichenette)")
    print("  [R] : Reset")
    
    demo_pendule_inverse_auto()