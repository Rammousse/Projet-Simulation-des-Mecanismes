import pygame
from math import pi
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers
from turtlebot import TurtleBot
from moteurCC import MoteurCC
from control_PID import ControlPID_vitesse

class TurtleBotMotorise(TurtleBot):
    """
    Robot où les roues sont pilotées par des moteurs CC
    asservis en vitesse via un PID.
    """
    def __init__(self, P0=V3D(), R0=0, radius=0.05, wheelbase=0.2, name='BotMotor', color='blue'):
        super().__init__(P0, R0, name, color)
        
        self.radius = radius        # Rayon des roues (m)
        self.wheelbase = wheelbase  # Entraxe roues (m)
        
        # 2 moteurs identiques pour la gauche et la droite
        params_moteur = {'R':1.0, 'L':0.001, 'ke':0.01, 'kc':0.01, 'J':0.01, 'f':0.1}
        
        self.motorL = MoteurCC(name="MotGauche", **params_moteur)
        self.motorR = MoteurCC(name="MotDroit",  **params_moteur)
        
        # PID pour chaque moteur pour maintenir la vitesse de consigne
        self.pidL = ControlPID_vitesse(self.motorL, Kp=5.0, Ki=20.0, Kd=0.0)
        self.pidR = ControlPID_vitesse(self.motorR, Kp=5.0, Ki=20.0, Kd=0.0)
        
        # consignes de vitesse de rotation des roues (rad/s)
        self.target_wL = 0.0
        self.target_wR = 0.0

    def set_speed_targets(self, w_left, w_right):
        """Définit la consigne de vitesse pour les régulateurs PID"""
        
        self.target_wL = w_left
        self.target_wR = w_right
        self.pidL.setTarget(w_left)
        self.pidR.setTarget(w_right)

    def simulate(self, step):
        
        self.pidL.simule(step)
        self.pidR.simule(step)
        
        real_wL = self.motorL.getSpeed()
        real_wR = self.motorR.getSpeed()
        
        # vitesse Linéaire V = R * (W_droite + W_gauche) / 2
        v_lin = self.radius * (real_wR + real_wL) / 2.0
        
        # vitesse Angulaire Omega = R * (W_droite - W_gauche) / L
        v_rot = self.radius * (real_wR - real_wL) / self.wheelbase
        
        self.speedTrans = v_lin
        self.speedRot = v_rot
        
        super().move(step)

    def gameDraw(self, scale, screen):
        super().gameDraw(scale, screen)
        
        X = int(scale * self.position.x)
        Y = int(scale * self.position.y)
        
        font = pygame.font.Font('freesansbold.ttf', 12)
        
        # # Affichage voir les consignes vs [réel]
        info_L = f"G:{self.target_wL:.1f} [{self.motorL.getSpeed():.1f}]"
        info_R = f"D:{self.target_wR:.1f} [{self.motorR.getSpeed():.1f}]"
        
        surf_L = font.render(info_L, True, (0, 0, 0))
        surf_R = font.render(info_R, True, (0, 0, 0))
        
        surf_L = pygame.transform.flip(surf_L, False, True)
        surf_R = pygame.transform.flip(surf_R, False, True)
        
        screen.blit(surf_L, (X - 40, Y - 30))
        screen.blit(surf_R, (X - 40, Y - 45))

def keyboard_control_motors(self, events, keys):
    """
    Contrôle clavier : on modifie les CONSIGNES des PIDs.
    """
    if not self.population: return
    bot = self.population[0] # On contrôle le premier robot
    
    increment = 1.0 # rad/s
    
    wl = bot.target_wL
    wr = bot.target_wR
    
    # Roue Gauche (Z/S)
    if keys[pygame.K_z] or keys[pygame.K_w]: wl += increment
    if keys[pygame.K_s]: wl -= increment
        
    # Roue Droite (Haut/Bas)
    if keys[pygame.K_UP]: wr += increment
    if keys[pygame.K_DOWN]: wr -= increment
    
    # Arrêt d'urgence (Espace)
    if keys[pygame.K_SPACE]:
        wl = 0
        wr = 0
        
    # Application des nouvelles consignes
    bot.set_speed_targets(wl, wr)


if __name__ == '__main__':
    
    monde = Univers(name="Arène Motorisée", dimensions=(10,10), gameDimensions=(1024,780), fps=60)
    robot = TurtleBotMotorise(P0=V3D(5, 5), radius=0.05, wheelbase=0.3, name="Wall-E", color="orange")
    
    monde.addParticule(robot)
    monde.gameInteraction = MethodType(keyboard_control_motors, monde)
    
    print("Contrôles :")
    print("  - Roue GAUCHE : 'Z' (avancer) / 'S' (reculer)")
    print("  - Roue DROITE : 'Flèche HAUT' / 'Flèche BAS'")
    print("  - ESPACE      : STOP")
    
    monde.game = True
    monde.simulateRealTime()