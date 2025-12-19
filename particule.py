from vector3D import Vector3D as V3D
from math import pi,atan2

class Particule(object):

    def __init__(self, mass=1, p0=V3D(), v0=V3D(), a0=V3D(), fix=False, name="paf", color='red'):
        self.mass = mass
        self.position = [p0]
        self.speed = [v0]
        self.acceleration = [a0]
        self.name = name
        self.color = color
        self.forces = V3D()
        self.fix = fix

    def __str__(self):
        msg = 'Particule ('+str(self.mass)+', '+str(self.position[-1])+', '+str(self.speed[-1])+', '+str(self.acceleration[-1])+', "'+self.name+'", "'+str(self.color)+'" )'
        return msg

    def __repr__(self):
        return str(self)

    def applyForce(self, *args):
        for f in args:
            self.forces += f

    def simulate(self,step):
        self.pfd(step)
        
    def pfd(self, step):
        
        if not(self.fix):
            a = self.forces * (1/self.mass)
            v = self.speed[-1]+a*step
        else :
            a = V3D()
            v = V3D()

        p = self.position[-1]+0.5*a*step**2 + self.speed[-1]*step

        self.acceleration.append(a)
        self.speed.append(v)
        self.position.append(p)
        self.forces = V3D()

    def plot(self):
        from pylab import plot
        X=[]
        Y=[]
        for p in self.position:
            X.append(p.x)
            Y.append(p.y)
    
        return plot(X,Y,color=self.color,label=self.name)+plot(X[-1],Y[-1],'o',color=self.color)    

    def getPosition(self):
        return self.position[-1]
    
    def getSpeed(self):
        return self.speed[-1]
    
    def gameDraw(self,scale,screen):
        import pygame
        
        X = int(scale*self.getPosition().x)
        Y = int(scale*self.getPosition().y)
        
        VX = int(scale*self.getSpeed().x)
        VY = int(scale*self.getSpeed().y) 
        size=3
        
        if type(self.color) is tuple:
            color = (self.color[0]*255,self.color[1]*255,self.color[2]*255)
        else:
            color=self.color
            
        pygame.draw.circle(screen,color,(X,Y),size*2,size)
        pygame.draw.line(screen,color,(X,Y),(X+VX,(Y+VY)),size)

if __name__=='__main__':
    from pylab import figure, show, legend

    P0 =Particule(v0=V3D(10,10,0))
    print(P0)
    
    while P0.getPosition().y >= 0. :
        P0.applyForce(V3D(0, -9.81, 0))
        
        
        if P0.getPosition().x >= 10:
            P0.applyForce(V3D(-6,0,0))
            
        P0.pfd(0.01)
        
    figure()
    P0.plot()
    legend()
    show()
