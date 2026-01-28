from random import random,randint
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from types import MethodType
from barre2D import Barre2D
from math import cos, sin, atan2

class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population = []
        self.generators = []
        self.step = step
        
        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale =  gameDimensions[0] / dimensions[0]
        
    
    def __str__(self):
        return 'Univers (%s,%g,%g)' % (self.name, self.time[0], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addParticule(self,*members):
        for i in members:
            self.population.append(i)
        
    def addGenerators(self,*members):
        for i in members:
            self.generators.append(i)
        
        
        
    def simulateAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            for source in self.generators :
                source.setForce(p)
            p.simulate(self.step)
        
        #for robot in self.robots:
        #    robot.move(self, step)
        
        self.time.append(self.time[-1]+self.step)

    def simulateFor(self,duration):
        # On calcule autant de pas que nécessaire pendant duration
        while duration > 0:
            self.simulateAll()
            duration -= self.step
        
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
    
    def gameInteraction(self,events,keys):
        # Fonctin qui sera surchargée par le client pour définir ses intéractions
        pass
    
    def simulateRealTime(self):
        # initilisation de l'environnement pygmae, création de la fenetre
        import pygame
        
        running = self.game
    
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
                
        # début simulation
        while running:
            screen.fill((240,240,240)) # effacer les images du pas précédent
            
            pygame.event.pump() # process event queue
            keys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
            events = pygame.event.get()
            
            # gestion de la fermeture de la fenetre / touche Echap
            if keys[pygame.K_ESCAPE]:
                running = False
                
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
            
            # Allons gérer les interactions ailleurs
            self.gameInteraction(events,keys) 
            
            # simuler les mouvement des chaque agent pendant la durée de ce pas
            self.simulateFor(1/self.gameFPS)    
            
            # demander à chaque agent sondessin en pixels sur la fenêtre
            for t in self.population:
                t.gameDraw(self.scale,screen)
            
            # dessin des ressorts
            for g in self.generators:
                if isinstance(g, SpringDamper):
                    g.gameDraw(self.scale, screen)
            
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            font_obj = pygame.font.Font('freesansbold.ttf', 22)
            text_surface_obj = font_obj.render(('time: %.2f' % self.time[-1]), True, 'black', (240,240,240))
            text_rect_obj = text_surface_obj.get_rect()
            text_rect_obj.topleft = (0, 0)
            
            screen.blit(text_surface_obj, text_rect_obj)
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()


class Force(object):
    
    def __init__(self,force=V3D(),name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        
    def __str__(self):
        return "Force ("+str(self.force)+', '+self.name+")"
        
    def __repr__(self):
        return str(self)

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.force)

class ForceSelect(Force):
    
    def __init__(self,force=V3D(),subject=None,name='force',active=True):
        self.force = force
        self.name = name
        self.active = active
        self.subjects=subject

    def setForce(self,particule):
        if self.active and particule in self.subjects:
            particule.applyForce(self.force)

class Gravity(Force):
    def __init__(self, g=V3D(0,-9.8), name='gravity', active=True):
        super().__init__(g, name, active) # Utilisation de super() conseillée

    def setForce(self, particule):
        if self.active:
            # Gravité s'applique au centre de masse (Point=0 pour Barre)
            if isinstance(particule, Barre2D):
                particule.applyEffort(Force=self.force * particule.mass, Point=0)
            else:
                # Cas Particule standard
                particule.applyForce(self.force * particule.mass)


class Effort(ForceSelect):
    """ Remplace Force et ForceSelect pour gérer Force ET Couple """
    def __init__(self, force=V3D(), torque=V3D(), point_app=0, subject=None, name='effort', active=True):
        super().__init__(force, subject, name, active)
        self.torque = torque
        self.point_app = point_app # Entre -1 et 1 pour une barre

    def setForce(self, p):
        # On vérifie si p est dans les sujets (si subjects est défini)
        if self.subjects is not None:
            if isinstance(self.subjects, list):
                if p not in self.subjects: return
            elif p != self.subjects: return

        if self.active:
            if isinstance(p, Barre2D):
                p.applyEffort(Force=self.force, Torque=self.torque, Point=self.point_app)
            else:
                p.applyForce(self.force)

class Bounce_y(Force):
    def __init__(self,k=1,step=0.1,name="boing",active=True):
        self.name=name
        self.k = k
        self.step = step

    def setForce(self,particule):
        if particule.getPosition().y < 0 and particule.getSpeed().y <0 :
            particule.applyForce(-2*(self.k/self.step)*V3D(0,particule.getSpeed().y * particule.mass ))
        
class Bounce_x(Force):
    def __init__(self,k=1,step=0.1,name="boing",active=True):
        self.name=name
        self.k = k
        self.step = step
        
    def setForce(self,particule):
        if particule.getPosition().x < 0 and particule.getSpeed().x <0 :
            particule.applyForce(-2*(self.k/self.step)*V3D(particule.getSpeed().x * particule.mass))

class BoxBoundaries(Force):
    def __init__(self, x_min, x_max, y_min, y_max, k=2000, damping=100, name="Boite"):
        super().__init__(V3D(), name, True)
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.k, self.damping = k, damping
    
    def setForce(self, p):
        if isinstance(p, Barre2D):
            pos, vit = p.position, p.speed
        else:
            pos, vit = p.getPosition(), p.getSpeed()
        
        # Mur Gauche / Droit
        if pos.x < self.x_min:
            p.applyForce(V3D(self.k * (self.x_min - pos.x) - self.damping * vit.x, 0, 0))
        elif pos.x > self.x_max:
            p.applyForce(V3D(self.k * (self.x_max - pos.x) - self.damping * vit.x, 0, 0))
        # Sol / Plafond
        if pos.y < self.y_min:
             p.applyForce(V3D(0, self.k * (self.y_min - pos.y) - self.damping * vit.y, 0))
        elif pos.y > self.y_max:
             p.applyForce(V3D(0, self.k * (self.y_max - pos.y) - self.damping * vit.y, 0))

class SpringDamper(Force):
    """ Version modifiée pour gérer les points d'ancrage sur Barre2D """
    def __init__(self, P0, P1, k=0, c=0, l0=0, active=True, name="spring", 
                anchor0=0, anchor1=0):
        super().__init__(V3D(), name, active)
        self.P0 = P0
        self.P1 = P1
        self.k = k
        self.c = c
        self.l0 = l0
        # anchor : 0 pour une particule, [-1, 1] pour une barre
        self.anchor0 = anchor0 
        self.anchor1 = anchor1
        self.visible = True
    
    def setForce(self, particule):
        # Récupération des positions et vitesses absolues
        # On utilise des helpers si c'est une barre, sinon direct
        if isinstance(self.P0, Barre2D):
            pos0 = self.P0.getWorldPoint(self.anchor0)
            vit0 = self.P0.getSpeedAtPoint(self.anchor0)
        else:
            pos0 = self.P0.getPosition()
            vit0 = self.P0.getSpeed()
            
        if isinstance(self.P1, Barre2D):
            pos1 = self.P1.getWorldPoint(self.anchor1)
            vit1 = self.P1.getSpeedAtPoint(self.anchor1)
        else:
            pos1 = self.P1.getPosition()
            vit1 = self.P1.getSpeed()
        
        vec_dir = pos1 - pos0
        dist = vec_dir.mod()
        
        if dist == 0: return # Évite division par zéro
        
        v_n = vec_dir.norm()
        elongation = dist - self.l0
        
        v_rel = vit1 - vit0
        v_proj = v_rel ** v_n # Produit scalaire
        amortissement = v_proj * self.c
        
        norme_force = (self.k * elongation) + amortissement
        force_vec = v_n * norme_force
        
        # Application
        if particule == self.P0:
            if isinstance(particule, Barre2D):
                particule.applyEffort(Force=force_vec, Point=self.anchor0)
            else:
                particule.applyForce(force_vec)
                
        elif particule == self.P1:
            if isinstance(particule, Barre2D):
                particule.applyEffort(Force=-force_vec, Point=self.anchor1)
            else:
                particule.applyForce(-force_vec)
    
    def gameDraw(self, scale, screen):
        if not self.active: return
        
        # trouve les positions exactes des extrémités en coordonnées monde
        # On réutilise la logique de setForce
        if isinstance(self.P0, Barre2D):
            pos0 = self.P0.getWorldPoint(self.anchor0)
        else:
            pos0 = self.P0.getPosition()
            
        if isinstance(self.P1, Barre2D):
            pos1 = self.P1.getWorldPoint(self.anchor1)
        else:
            pos1 = self.P1.getPosition()
        
        # conversion en pixels
        x0 = int(pos0.x * scale)
        y0 = int(pos0.y * scale)
        x1 = int(pos1.x * scale)
        y1 = int(pos1.y * scale)
        
        # dessin
        pygame.draw.line(screen, (0, 255, 0), (x0, y0), (x1, y1), 2)

class TorsionSpringDamper(Force):
    """ Ressort angulaire : C = -k*(theta - theta0) - c*omega """
    def __init__(self, target, k=1.0, c=0.1, theta0=0.0, name="Torsion"):
        super().__init__(V3D(), name, True)
        self.target = target # La barre concernée
        self.k = k
        self.c = c
        self.theta0 = theta0

    def setForce(self, p):
        # On n'applique la force que si c'est la bonne cible
        if p == self.target and isinstance(p, Barre2D):
            # Calcul du couple
            couple_rappel = -self.k * (p.theta - self.theta0)
            couple_amorti = -self.c * p.omega
            
            C_total = couple_rappel + couple_amorti
            
            # Application (Point=0 car couple pur)
            p.applyEffort(Torque=V3D(0, 0, C_total), Point=0)

class LiaisonPivot(SpringDamper):
    """ Force deux points à coïncider (Ressort raide, l0=0) """
    def __init__(self, P0, P1, anchor0=0, anchor1=0, k=5000, c=50, name="Pivot"):
        # l0 = 0 pour que les points se touchent
        super().__init__(P0, P1, k, c, l0=0, active=True, name=name, 
                        anchor0=anchor0, anchor1=anchor1)


class LiaisonGlissiere(Force):
    """
    Maintient la particule sur un axe horizontal (y = constante).
    """
    def __init__(self, particule, y_fixe, k=4000, c=150):
        super().__init__(name="Glissiere")
        self.particule = particule
        self.y_fixe = y_fixe
        self.k = k
        self.c = c
    
    def setForce(self, p):
        if p != self.particule: return
        
        pos = p.getPosition()
        vit = p.getSpeed()
        
        # Rappel vers la ligne Y fixe pour le ressort vertical
        dy = pos.y - self.y_fixe
        vy = vit.y
        fy = -self.k * dy - self.c * vy
        
        # Rappel vers Z=0 pour éviter que ça parte en profondeur
        dz = pos.z
        vz = vit.z
        fz = -self.k * dz - self.c * vz
        
        p.applyForce(V3D(0, fy, fz))

class Link(SpringDamper):
    def __init__(self,P0,P1,name="link"):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,5000,100,l0,True,name)

class Prism(SpringDamper):
    def __init__(self,P0,P1,axis=V3D(),name="prism"):
        l0 = (P0.getPosition()-P1.getPosition()).mod()
        SpringDamper.__init__(self,P0, P1,1000,100,l0,True,name)
        self.axis=axis.norm()

    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
        vec_dir -= vec_dir ** self.axis * self.axis
        v_n = vec_dir.norm()
        flex = vec_dir.mod()-self.l0
        
        vit = self.P1.getSpeed() - self.P0.getSpeed()
        vit_n = vit ** v_n * self.c 
        
        force = (self.k * flex + vit_n)* v_n
        if particule == self.P0:
            particule.applyForce(force)
        elif particule == self.P1:
            particule.applyForce(-force)
        else:
            pass
    
class Viscosity(Force):
    def __init__(self, coeff=0,name='viscosity',active=True):
        self.coeff = coeff
        self.name = name
        self.active = active

    def setForce(self,particule):
        if self.active:
            viscosity = particule.getSpeed() * -self.coeff
            particule.applyForce(viscosity)

class Pichenette(Effort):
    """
    Force temporaire qui s'applique pendant une durée définie puis se désactive.
    """
    def __init__(self, target, force_vector, duration=0.2, step=0.001):
        # On initialise comme un Effort (Force ponctuelle sur une barre)
        super().__init__(force=force_vector, subject=target, point_app=1, name="Pichenette")
        
        # Point=1 signifie le sommet du pendule
        
        self.duration = duration
        self.step = step
        self.elapsed = 0.0
        
    def setForce(self, p):
        # Si la force n'est plus active, on ne fait rien
        if not self.active:
            return
        
        # On laisse la classe mère (Effort) appliquer la force physique
        super().setForce(p)
        
        if p == self.subjects:
            self.elapsed += self.step
            
            # Si le temps est écoulé, on désactive la force
            if self.elapsed >= self.duration:
                self.active = False


class LiaisonMotorisee(Force):
    def __init__(self, bodyA, bodyB, motor, pid, dt, anchorA=0, anchorB=0):
        super().__init__(name=f"Joint_{motor.name}")
        self.bodyA = bodyA 
        self.bodyB = bodyB 
        self.motor = motor
        self.pid = pid
        self.dt = dt
        self.voltage = 0.0
        self.anchorA = anchorA
        self.anchorB = anchorB
    
    def setForce(self, p):
        # On applique les forces uniquement lors du traitement du corps B (le bras)
        # pour éviter de doubler les calculs ou l'intégration du PID.
        if p != self.bodyB: 
            return
        
        # Récupération de l'état cinématique (angles et vitesses)
        # Si le corps A est une barre, on récupère ses valeurs, sinon on considère qu'il est fixe (0)
        if isinstance(self.bodyA, Barre2D):
            theta_A = self.bodyA.theta
            omega_A = self.bodyA.omega
        else:
            theta_A = 0
            omega_A = 0
        
        theta_B = self.bodyB.theta
        omega_B = self.bodyB.omega
        
        relative_theta = theta_B - theta_A
        relative_omega = omega_B - omega_A
        
        # Calcul de la tension de commande via le PID
        pid_voltage = self.pid.compute(relative_theta, self.dt)
        
        # Compensation de gravité (Feed-Forward)
        m = self.bodyB.mass
        L = self.bodyB.length 
        g = 9.81
        
        # Estimation du couple nécessaire pour compenser le poids du bras
        C_gravity = m * g * (L/2.0) * cos(theta_B)
        
        # Conversion du couple de gravité en tension équivalente pour aider le moteur
        u_gravity = (self.motor.R * C_gravity) / self.motor.kc
        
        # Calcul de la tension totale avec saturation (limites de l'alimentation)
        self.voltage = pid_voltage + u_gravity
        self.voltage = max(-12.0, min(12.0, self.voltage))
        
        # Modèle électrique du moteur (U = E + RI)
        E = self.motor.ke * relative_omega
        I = (self.voltage - E) / self.motor.R
        Gamma = self.motor.kc * I
        
        TorqueVec = V3D(0, 0, Gamma)
        
        # Action moteur sur le bras
        self.bodyB.applyEffort(Torque=TorqueVec, Point=self.anchorB)
            
        # Réaction sur le support (uniquement si c'est un objet mobile comme une barre)
        if isinstance(self.bodyA, Barre2D):
            self.bodyA.applyEffort(Torque=-TorqueVec, Point=self.anchorA)