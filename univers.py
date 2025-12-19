from random import random,randint
from vector3D import Vector3D as V3D
from particule import Particule
import pygame
from pygame.locals import *
from types import MethodType

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
    def __init__(self,g=V3D(0,-9.8),name='gravity',active=True):
        self.g = g
        self.name = name
        self.active = active

    def setForce(self,particule):
        if self.active:
            particule.applyForce(self.g*particule.mass)


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
        
class SpringDamper(Force):
    def __init__(self,P0,P1,k=0,c=0,l0=0,active=True,name="spring_and_damper"):
        Force.__init__(self,V3D(),name,active)
        self.k = k
        self.c = c
        self.P0 = P0
        self.P1 = P1
        self.l0 = l0
    
    def setForce(self, particule):
        vec_dir = self.P1.getPosition() - self.P0.getPosition()
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

if __name__=='__main__':
    from pylab import figure, show, legend
    
    monUnivers = Univers(game=True, fps=60)
    
    monUnivers.step=0.001
    
    P0 = Particule(p0=V3D(10,10,0))
    P_fixe = Particule(p0=V3D(monUnivers.dimensions[0]/2,monUnivers.dimensions[1]/2),fix=True, color='black')
    P_osc = Particule(p0=V3D(monUnivers.dimensions[0]/2 - 10,monUnivers.dimensions[1]/2 - 10))
    
    force_gravity = Gravity(V3D(0,-10))
    viscous = Viscosity(coeff=0)
    
    boing = Bounce_y(.9,monUnivers.step) 
    boing2 = Bounce_x(1,monUnivers.step) 
    
    ressort = SpringDamper(P_fixe,P_osc,k=10,l0= 10 , c=1)
    
    monUnivers.addParticule(P_fixe, P_osc)
    monUnivers.addParticule(P0)
    
    monUnivers.addGenerators(viscous, force_gravity)
    monUnivers.addGenerators(boing,boing2,ressort)
    
    def myInteraction(self,events,keys):
        # controle de leader avec le clavier
        if keys[ord('z')] or keys[pygame.K_UP]: # And if the key is z or K_DOWN:
            P_osc.applyForce(V3D(0,15))
        if keys[ord('s')] or keys[pygame.K_DOWN]: # And if the key is s or K_DOWN:
            P_osc.applyForce(V3D(0,-15))
        if keys[ord('q')] or keys[pygame.K_LEFT]: # And if the key is q or K_DOWN:
            P_osc.applyForce(V3D(-15,0))
        if keys[ord('d')] or keys[pygame.K_RIGHT]: # And if the key is d K_DOWN:
            P_osc.applyForce(V3D(15,0))
        
        if keys[pygame.K_SPACE]:
            force_gravity.active = not force_gravity.active
        
        # Création des particules au clic de souris 
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                x , y = event.pos # les coordonnées en pixel, y vers le bas !
                pos = V3D(x/self.scale,(monUnivers.gameDimensions[1]-y)/self.scale) # il faut mettre l'axe y vers le haut! 
                vit = V3D(random()*20-10,random()*20-10)
                name='P_'+str(len(monUnivers.population))
                color=(random(),random(),random())
                part = Particule(p0=pos,v0=vit,name=name,color=color,mass=1)
                monUnivers.addParticule(part)

# Surcharge de la fonction ici
    monUnivers.gameInteraction = MethodType(myInteraction,monUnivers)
    
    monUnivers.simulateRealTime()
    
    monUnivers.plot()
