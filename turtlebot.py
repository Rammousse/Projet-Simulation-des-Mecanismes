from vector3D import Vector3D as V3D
from math import pi,atan2
from random import random,randint

# un objet TurtleBot pour une simulation cinématique simple 
class TurtleBot(object):
    def __init__(self,P0=V3D(),R0=0,name='toto',color='red'):
        
        self.position=P0
        self.orientation=R0
        self.pose=[(P0,R0)]
        self.name=name
        self.color=color
        
        self.speedTrans = 0
        self.speedRot = 0
        
        self.speedTransMax = 3
        self.speedRotMax = pi/4
        
    def __str__(self):
        return "Turtle (%s, %g, %s, %s)" % (self.position, self.orientation, self.name, self.color)
    
    def __repr__(self):
        return str(self)
    
    def turn(self,angle):
        self.orientation = self.orientation + angle
        self.pose.append((self.position,self.orientation))
                
    def walk(self,dist):
        d = V3D(dist,0,0).rotZ(self.orientation)
        self.position = self.position + d
        self.pose.append((self.position,self.orientation))
    
    def move(self,step=0.1):
        # calcul du déplacement (rotation puis translation) en fct des vitesses
        angle = self.speedRot * step
        dist = self.speedTrans * step
        self.turn(angle)
        self.walk(dist)
        
    def controlGoTo(self,target=V3D(),Kp=10):
        # Correcteur proportionel: on calcule des vitesses pour aller vers target
        errorVect = target-self.position
        targetDirection = errorVect.norm()
        posError = errorVect.mod()

        turtleDirection = V3D(1,0).rotZ(self.orientation)       
        rotError = ( turtleDirection * targetDirection).z #sinus de l'angle entre les 2 vect, en conservant le signe

        if posError > 0.01:
            self.speedTrans = Kp * posError
            self.speedTrans = self.speedTrans * (targetDirection ** turtleDirection) # On diminue la vitesse désirée si on n'est pas vers la cible

            if self.speedTrans > self.speedTransMax:
                self.speedTrans = self.speedTransMax
            if self.speedTrans  < 0: 
                self.speedTrans  = 0 # pas de marche arrière

        else: 
            self.speedTrans = 0
        
        self.speedRot = Kp * rotError

        if self.speedRot > self.speedRotMax:
            self.speedRot = self.speedRotMax
        if self.speedRot < -self.speedRotMax:
            self.speedRot = - self.speedRotMax
        
    def plot(self):
        
        from pylab import plot
        X=[]
        Y=[]
        for p in self.pose:
            X.append(p[0].x)
            Y.append(p[0].y)
            
    
        return plot(X,Y,color=self.color,label=self.name)+plot(X[-1],Y[-1],'*',color=self.color)   
        
    def gameDraw(self,scale,screen):
        # on va calculer les position en pixel et dessiner eds objet pygame sur la fenetre
        import pygame
        
        X = int(scale*self.position.x)
        Y = int(scale*self.position.y)
        
        vit = V3D(self.speedTrans).rotZ(self.orientation)
        VX = int(scale*vit.x)
        VY = int(scale*vit.y) 
        size=5
        
        if type(self.color) is tuple:
            color = (self.color[0]*255,self.color[1]*255,self.color[2]*255)
        else:
            color=self.color
            
        pygame.draw.circle(screen,color,(X,Y),size*2,size)
        pygame.draw.line(screen,color,(X,Y),(X+VX,(Y+VY)),size)


class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population=[]
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
        
    def addUnit(self,*members):
        for i in members:
            self.population.append(i)
        
    def stepAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            p.move(self.step)
        self.time.append(self.time[-1]+self.step)

    def moveAll(self,duration):
        # On calcule autant de pas que nécessaire pendant duration
        while duration > 0:
            self.stepAll()
            duration -= self.step
    
    
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()

    def gameDrawAll(self,screen):
        
        for p in self.population:
            p.gameDraw(self.scale,screen)
        
            

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
            self.moveAll(1/self.gameFPS)    
            
            # demander à chaque agent sondessin en pixels sur la fenêtre
            self.gameDrawAll(screen)
            
            
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()

if __name__ == "__main__":    
    from pylab import figure, show, legend
    import pygame
    from pygame.locals import *
    from types import MethodType
    
    step = 0.1
    
    bob = TurtleBot(P0=V3D(50,50),name='bob')
    toto = TurtleBot(P0=V3D(10,50), color='green')
    print(toto)
    print(bob)
    # bob.turn(pi/4)
    # bob.walk(.5)
    # bob.turn(pi/4)
    # bob.walk(.3)
    # toto.turn(pi/2)
    # toto.walk(1)
    # toto.turn(-pi/4)
    # toto.walk(.8)

    bob.speedTrans = 1
    bob.speedRot = pi/10
    
    """for i in range(100):
        bob.controlGoTo(V3D(-25,10))
        bob.move(step)
        
    bob.speedTrans = .5
    bob.speedRot = -pi/20
    
    for i in range(500):
        bob.move(step)

        toto.controlGoTo(V3D(-25,25))
        toto.move()
        
    for i in range(100):
        bob.controlGoTo(V3D(-25,0),1)
        bob.move(step)
    
    figure('Turtles')
    bob.plot()
    toto.plot()
    legend()
    show()

    plage = Univers(name='plage',game=True)
    
    plage.addUnit(toto,bob)
    
    plage.simulateRealTime()

    plage.plot()"""
    
    # Création de l'environnement de simulation
    
    plage = Univers(name='plage',dimensions=(20,20))

    # On crée N robot aléatoires
    N = 5
    for i in range(N):
        x = random()*10
        y =  random()*10
        r = 2 * random() * pi
        p0 = V3D(x,y)
        name = 'Tortue'+str(i)
        color=(random(),random(),random())
        t = TurtleBot(p0,r,name,color=color)
        t.speedTrans = random()*.5    
        t.speedRot = random()*pi/20
        plage.addUnit(t)

    #On désigne la tortue 0 comme leader (=bob)
    bob = plage.population[0]
    bob.position=V3D(10,10)
    bob.speedRot = 0
    bob.speedTrans = 0.3
    bob.orientation=pi/4
    
    def myInteraction(self, events, keys):
        # controle de leader avec le clavier
        if keys[ord('z')] or keys[pygame.K_UP]: # And if the key is z or K_DOWN:
            bob.speedTrans += .05
        if keys[ord('s')] or keys[pygame.K_DOWN]: # And if the key is s or K_DOWN:
            bob.speedTrans -= .05
        if keys[ord('q')] or keys[pygame.K_LEFT]: # And if the key is q or K_DOWN:
            bob.orientation += pi/50
        if keys[ord('d')] or keys[pygame.K_RIGHT]: # And if the key is d K_DOWN:
            bob.orientation -= pi/50
        
        for turtle in plage.population[1:]:
            turtle.controlGoTo(bob.position,.5) 
    
    """plage.gameInteraction = MethodType(myInteraction,plage)
    
    # Lancement de la simulation
    plage.game=True
    plage.simulateRealTime()

    plage.plot()"""
    
    def clickInteraction(self, events, keys):
        
        for event in events:
            if event.type == pygame.MOUSEBUTTONDOWN:
                x , y = event.pos
                # Conversion des coordonnées pixels -> simulation
                # On inverse Y car Pygame a (0,0) en haut à gauche
                real_x = x / self.scale
                real_y = (self.gameDimensions[1] - y) / self.scale
                
                # On met à jour la cible mémorisée par Bob
                bob.target = V3D(real_x, real_y)
                
        bob.controlGoTo(bob.target)
        
        for turtle in plage.population[1:]:
            turtle.controlGoTo(bob.position,.5)
    
    bob.target = bob.position
    plage.gameInteraction = MethodType(clickInteraction,plage)
    
    # Lancement de la simulation
    plage.game=True
    plage.simulateRealTime()

    plage.plot()

