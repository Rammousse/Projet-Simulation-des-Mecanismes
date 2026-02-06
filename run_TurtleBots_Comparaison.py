import pygame
import matplotlib.pyplot as plt
from math import pi, sqrt
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers
from run_TurtleBots_Wheels import TurtleBotDifferential
from run_TurtleBots_Motors import TurtleBotMotorise


PID_KP = 100.0   
PID_KI = 60.0  
PID_KD = 20.0   

KP_GUIDAGE = 2.0  
V_MAX = 1.0       

DT = 0.01
FPS = 60

def compute_inverse_kinematics(robot, v_lin_desire, v_rot_desire):
    """Calcule les consignes roues (Gauche, Droite) pour atteindre (v, omega)."""
    R = robot.radius
    L = robot.wheelbase
    w_right = (v_lin_desire + (L/2) * v_rot_desire) / R
    w_left  = (v_lin_desire - (L/2) * v_rot_desire) / R
    return w_left, w_right

def control_strategy_goto(robot, target, Kp=1.0, v_max=1.0):
    """Stratégie de guidage vers un point."""
    errorVect = target - robot.position
    dist = errorVect.mod()
    
    targetDir = errorVect.norm()
    turtleDir = V3D(1,0,0).rotZ(robot.orientation)
    
    rotError = (turtleDir * targetDir).z 
    
    w_cmd = 4.0 * rotError 
    w_max = pi
    if w_cmd > w_max: w_cmd = w_max
    if w_cmd < -w_max: w_cmd = -w_max
    
    if dist > 0.05:
        alignment = (targetDir ** turtleDir)
        if alignment > 0:
            v_cmd = Kp * dist * alignment
        else:
            v_cmd = 0 
        if v_cmd > v_max: v_cmd = v_max
    else:
        v_cmd = 0
        w_cmd = 0
        
    return v_cmd, w_cmd

def my_interaction(self, events, keys):
    # Mise à jour cible au clic
    for event in events:
        if event.type == pygame.MOUSEBUTTONDOWN:
            x, y = event.pos
            real_x = x / self.scale
            real_y = (self.gameDimensions[1] - y) / self.scale
            self.target_point = V3D(real_x, real_y)
            print(f"Direction : {self.target_point}")
            
    for robot in self.population:
        v_des, w_des = control_strategy_goto(robot, self.target_point, Kp=KP_GUIDAGE, v_max=V_MAX)
        wl, wr = compute_inverse_kinematics(robot, v_des, w_des)
        
        if isinstance(robot, TurtleBotDifferential):
            robot.set_wheel_speeds(wl, wr)
        elif isinstance(robot, TurtleBotMotorise):
            robot.set_speed_targets(wl, wr)


class UniversComparaison(Univers):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.target_point = V3D(2, 2) 
        # Suppression de self.traces
        self.history = {} 
        
    def addParticule(self, robot):
        super().addParticule(robot)
        # On initialise seulement l'historique pour les graphs de fin
        self.history[robot] = {'t':[], 'v':[], 'x':[], 'y':[]}

    def record_data(self):
        """Enregistre les données pour les graphes"""
        t = self.time[-1]
        for robot in self.population:
            self.history[robot]['t'].append(t)
            self.history[robot]['v'].append(robot.speedTrans)
            self.history[robot]['x'].append(robot.position.x)
            self.history[robot]['y'].append(robot.position.y)

    def simulateRealTime(self):
        import pygame
        running = self.game
        pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
        
        while running:
            screen.fill((240,240,240))
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            events = pygame.event.get()
            if keys[pygame.K_ESCAPE]: running = False
            for event in events:
                if event.type == pygame.QUIT: running = False
            
            self.gameInteraction(events, keys) 
            self.simulateFor(1/self.gameFPS)    
            self.record_data() 
            
            self.gameDrawAll(screen)
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            pygame.display.flip()
            clock.tick(self.gameFPS)
        pygame.quit()

    def gameDrawAll(self, screen):
        # Dessin de la cible
        cx = int(self.scale * self.target_point.x)
        cy = int(self.scale * self.target_point.y)
        pygame.draw.circle(screen, (0, 200, 0), (cx, cy), 8) 
        
        # Dessin des robots uniquement (plus de traces)
        for robot in self.population:
            robot.gameDraw(self.scale, screen)

def plot_results_comparison(monde):
    plt.figure("Comparaison Trajectoires", figsize=(10, 5))
    
    # Trajectoire
    plt.subplot(1, 2, 1)
    plt.title("Trajectoires XY")
    for robot in monde.population:
        hist = monde.history[robot]
        label = "Idéal" if isinstance(robot, TurtleBotDifferential) else "Moteur CC"
        style = '--' if isinstance(robot, TurtleBotDifferential) else '-'
        # On trace toute l'histoire accumulée
        plt.plot(hist['x'], hist['y'], style, label=label, linewidth=2)
    
    plt.plot(monde.target_point.x, monde.target_point.y, 'gx', markersize=10, label='Cible')
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    # Vitesse
    plt.subplot(1, 2, 2)
    plt.title("Vitesse Linéaire vs Temps")
    for robot in monde.population:
        hist = monde.history[robot]
        label = "Idéal" if isinstance(robot, TurtleBotDifferential) else "Moteur CC"
        plt.plot(hist['t'], hist['v'], label=label)
    
    plt.xlabel("Temps (s)")
    plt.ylabel("Vitesse (m/s)")
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    largeur_monde = 10.0 
    W_fenetre = 1024
    scale = W_fenetre / largeur_monde
    H_fenetre = int(largeur_monde * 0.75 * scale) 
    
    monde = UniversComparaison(name="Comparaison Modèles", 
                               dimensions=(largeur_monde, largeur_monde * 0.75), 
                            gameDimensions=(W_fenetre, H_fenetre), 
                            fps=FPS, step=DT)
    
    start_pos = V3D(2, 2)
    
    robot_ideal = TurtleBotDifferential(P0=start_pos, radius=0.05, wheelbase=0.3, 
                                        name="Ideal (Cinématique)", color="blue")
    
    robot_dyn = TurtleBotMotorise(P0=start_pos, radius=0.05, wheelbase=0.3, 
                                name="Dynamique (Moteurs)", color="orange")
    
    robot_dyn.pidL.Kp = PID_KP
    robot_dyn.pidL.Ki = PID_KI 
    robot_dyn.pidR.Kp = PID_KP
    robot_dyn.pidR.Ki = PID_KI
    
    monde.addParticule(robot_ideal)
    monde.addParticule(robot_dyn)
    
    monde.gameInteraction = MethodType(my_interaction, monde)
    
    print("Cliquez sur la fenêtre pour donner une destination.")
    
    monde.game = True
    monde.simulateRealTime()
    
    plot_results_comparison(monde)