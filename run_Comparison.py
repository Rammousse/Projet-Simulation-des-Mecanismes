import pygame
from math import pi
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers
from run_TurtleBots_Wheels import TurtleBotDifferential
from run_TurtleBots_Motors import TurtleBotMotorise

# PARAMÈTRES PID
PID_KP = 15.0
PID_KI = 10.0
PID_KD = 0.5

DT = 0.01
FPS = 60
V_MAX = 20.0

def compute_wheel_speeds(robot, v_lin, v_rot):
    """
    Cinématique Inverse : Transforme (Vitesse linéaire, Vitesse angulaire)
    en consignes (Roue Gauche, Roue Droite).
    """
    R = robot.radius
    L = robot.wheelbase
    
    # Formules du robot différentiel
    w_right = (2 * v_lin + v_rot * L) / (2 * R)
    w_left  = (2 * v_lin - v_rot * L) / (2 * R)
    
    return w_left, w_right

def interaction_click(self, events, keys):
    
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            
            real_x = x / self.scale
            real_y = (self.gameDimensions[1] - y) / self.scale
            
            self.target_point = V3D(real_x, real_y)
            print(f"Nouvelle cible : {self.target_point}")
        
    for robot in self.population:
        
        robot.speedTransMax = V_MAX
        robot.controlGoTo(self.target_point, Kp=2.0) # Kp de guidage
        
        v_desire = robot.speedTrans
        w_desire = robot.speedRot
        
        # conversion en vitesses de roues
        wl, wr = compute_wheel_speeds(robot, v_desire, w_desire)
        
        if isinstance(robot, TurtleBotDifferential):
            # Robot Cinématique
            robot.set_wheel_speeds(wl, wr)
            
        elif isinstance(robot, TurtleBotMotorise):
            # Robot Dynamique
            robot.set_speed_targets(wl, wr)


class UniversComparaison(Univers):
    """
    Univers spécialisé pour afficher la cible et les traces des robots.
    """
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.target_point = V3D(5, 5) # Cible par défaut au centre
        self.traces = {}              # Historique pour le dessin
        
    def addUnit(self, robot):
        super().addUnit(robot)
        self.traces[robot] = []

    def gameDrawAll(self, screen):
        cx = int(self.scale * self.target_point.x)
        cy = int(self.scale * self.target_point.y)
        pygame.draw.circle(screen, (255, 0, 0), (cx, cy), 5)      # Point
        pygame.draw.circle(screen, (255, 0, 0), (cx, cy), 15, 1)  # Cercle autour
        
        for robot in self.population:
            # on enregistre un point s'il a bougé un peu (pour éviter de saturer la mémoire)
            if len(self.traces[robot]) == 0 or (robot.position - self.traces[robot][-1]).mod() > 0.05:
                self.traces[robot].append(robot.position)
                
                # on garde seulement les 1000 derniers points
                if len(self.traces[robot]) > 1000: 
                    self.traces[robot].pop(0)
            
            # dessin de la ligne de trace
            if len(self.traces[robot]) > 1:
                pts_pixel = [(int(self.scale * p.x), int(self.scale * p.y)) for p in self.traces[robot]]
                
                color = (0, 0, 255) if isinstance(robot, TurtleBotDifferential) else (255, 140, 0)
                
                pygame.draw.lines(screen, color, False, pts_pixel, 2)
        
        super().gameDrawAll(screen)
        
        font = pygame.font.Font('freesansbold.ttf', 16)
        
        l1 = pygame.transform.flip(font.render("Clic Gauche : Définir cible", True, (50, 50, 50)), False, True)
        l2 = pygame.transform.flip(font.render("Bleu : Modèle Idéal", True, (0, 0, 255)), False, True)
        l3 = pygame.transform.flip(font.render(f"Orange : Modèle Dynamique (PID P={PID_KP})", True, (255, 100, 0)), False, True)
        
        screen.blit(l1, (10, self.gameDimensions[1] - 30))
        screen.blit(l2, (10, self.gameDimensions[1] - 55))
        screen.blit(l3, (10, self.gameDimensions[1] - 80))


if __name__ == '__main__':
    
    W, H = 1024, 780
    largeur_m = 10.0
    hauteur_m = largeur_m * H / W
    
    monde = UniversComparaison(name="Comparaison Click", dimensions=(largeur_m, hauteur_m), 
                            gameDimensions=(W, H), fps=FPS, step=DT)
    
    start_pos = V3D(2, 2)
    
    # Robot Cinématique (Bleu)
    robot_ideal = TurtleBotDifferential(P0=start_pos, radius=0.05, wheelbase=0.3, 
                                        name="Ideal", color="blue")
    
    # Robot Dynamique (Orange)
    robot_dyn = TurtleBotMotorise(P0=start_pos, radius=0.05, wheelbase=0.3, 
                                name="Dynamique", color="orange")
    
    robot_dyn.pidL.Kp = PID_KP
    robot_dyn.pidL.Ki = PID_KI
    robot_dyn.pidL.Kd = PID_KD
    robot_dyn.pidR.Kp = PID_KP
    robot_dyn.pidR.Ki = PID_KI
    robot_dyn.pidR.Kd = PID_KD
    
    monde.addParticule(robot_ideal)
    monde.addParticule(robot_dyn)
    
    monde.gameInteraction = MethodType(interaction_click, monde)
    
    monde.game = True
    print("=== Simulation Lancée ===")
    print("Cliquez sur l'écran pour définir une destination.")
    print(f"PID Actuel : P={PID_KP}, I={PID_KI}, D={PID_KD}")
    monde.simulateRealTime()