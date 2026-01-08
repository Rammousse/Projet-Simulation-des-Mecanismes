import pygame
from math import pi
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers
from turtlebot import TurtleBot

class TurtleBotDifferential(TurtleBot):
    """
    Version améliorée du TurtleBot avec un modèle cinématique différentiel (2 roues).
    La commande se fait via les vitesses de rotation des roues (rad/s).
    """
    def __init__(self, P0=V3D(), R0=0, radius=0.05, wheelbase=0.2, name='DiffBot', color='blue'):
        
        super().__init__(P0, R0, name, color)
        
        self.radius = radius        # rayon des roues (m)
        self.wheelbase = wheelbase  # distance entre les roues (m)
        
        # État des roues (Commandes)
        self.w_left = 0.0   # Vitesse roue gauche (rad/s)
        self.w_right = 0.0  # Vitesse roue droite (rad/s)

    def set_wheel_speeds(self, wl, wr):
        """Impose directement les vitesses de rotation des roues"""
        self.w_left = wl
        self.w_right = wr

    def move(self, step=0.1):
        """
        Surcharge de la méthode move.
        Calcule la vitesse linéaire et angulaire du robot à partir des roues,
        puis laisse la classe mère faire l'intégration de position.
        """
        # Vitesse Linéaire V = R * (W_droite + W_gauche) / 2
        v_lin = self.radius * (self.w_right + self.w_left) / 2.0
        
        # Vitesse Angulaire Omega = R * (W_droite - W_gauche) / L
        v_rot = self.radius * (self.w_right - self.w_left) / self.wheelbase
        
        # Mise à jour des attributs hérités de TurtleBot
        self.speedTrans = v_lin
        self.speedRot = v_rot
        
        # Appel de la méthode move originale pour appliquer le déplacement (x, y, theta)
        super().move(step)

    def simulate(self, step):
        """Méthode de compatibilité pour la classe Univers."""
        self.move(step)

    def gameDraw(self, scale, screen):
        """Surcharge pour afficher les infos des roues à l'écran de manière lisible"""
        
        super().gameDraw(scale, screen)
        
        X = int(scale * self.position.x)
        Y = int(scale * self.position.y)
        
        # Configuration du texte
        font = pygame.font.Font('freesansbold.ttf', 16)
        text_color = (0, 0, 0) # Noir pour le contraste
        text_content = f"L:{self.w_left:.1f} | R:{self.w_right:.1f}"
        
        # Création de la surface de texte brute
        text_surface_raw = font.render(text_content, True, text_color)
        
        text_surface_final = pygame.transform.flip(text_surface_raw, False, True)
        text_rect = text_surface_final.get_rect()
        
        # On place le milieu-bas du texte un peu en-dessous du centre du robot.
        text_rect.midbottom = (X, Y - 25) 
        
        # Affichage
        screen.blit(text_surface_final, text_rect)

def keyboard_control_wheels(self, events, keys):
    """Contrôle de type 'Tank'"""
    # Récupération du robot cible
    if not self.population: return
    
    # On cherche le robot différentiel dans la population (au cas où il y en aurait d'autres)
    bot = next((p for p in self.population if isinstance(p, TurtleBotDifferential)), None)
    if bot is None: return
    
    # sensibilité des touches
    increment = 0.5 # rad/s
    
    # Roue Gauche (Touches Z et S)
    if keys[pygame.K_w] or keys[pygame.K_z]: # Avancer Gauche
        bot.w_left += increment
    if keys[pygame.K_s]: # Reculer Gauche
        bot.w_left -= increment
        
    # Roue Droite (Flèches Haut/Bas)
    if keys[pygame.K_UP]: # Avancer Droite
        bot.w_right += increment
    if keys[pygame.K_DOWN]: # Reculer Droite
        bot.w_right -= increment
        
    # arrêt d'urgence (Espace)
    if keys[pygame.K_SPACE]:
        bot.w_left = 0
        bot.w_right = 0


if __name__ == '__main__':
    
    monde = Univers(name="Arène TurtleBot", dimensions=(5,5), gameDimensions=(1024,780), fps=60)
    robot = TurtleBotDifferential(P0=V3D(2.5, 2.5), radius=0.05, wheelbase=0.3, name="Wall-E", color="orange")
    
    monde.addParticule(robot)
    monde.gameInteraction = MethodType(keyboard_control_wheels, monde)
    
    print("Contrôles :")
    print("  - Roue GAUCHE : Touches 'Z' pour avancer, 'S' pour reculer")
    print("  - Roue DROITE : Flèche HAUT pour avancer, BAS pour reculer")
    print("  - ESPACE      : Stop")
    
    monde.game = True
    monde.simulateRealTime()