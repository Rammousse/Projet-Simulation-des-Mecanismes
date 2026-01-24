from vector3D import Vector3D as V3D
from math import cos, sin, pi
import pygame

class Barre2D(object):
    def __init__(self, mass=1, long=1, theta=0, pos=V3D(), fixed=False, color='red', nom='barre'):
        self.mass = mass
        self.length = long
        self.theta = theta
        self.position = pos
        self.fixed = fixed
        self.color = color
        self.name = nom
        
        # vitesse et accélération angulaires
        self.omega = 0
        self.alpha = 0
        
        # vitesse et accélération linéaires
        self.speed = V3D()
        self.acc = V3D()
        
        # accumulateurs d'efforts
        self.sum_forces = V3D()
        self.sum_moments = 0.0 # Scalaire en 2D (autour de Z)
        
        # moment d'inertie pour une tige fine tournant autour de son centre
        # J = (1/12) * M * L^2
        self.J = (1.0/12.0) * self.mass * (self.length**2)
    
    def getPosition(self):
        """ Retourne la position du centre de gravité """
        return self.position
    
    def getSpeed(self):
        """ Retourne la vitesse linéaire du centre de gravité """
        return self.speed
    
    def applyForce(self, force):
        """ 
        Applique une force simple sur le centre de gravité (Point 0).
        Permet à la barre de réagir à la Viscosité ou aux forces génériques.
        """
        self.applyEffort(Force=force, Point=0)
    
    def applyEffort(self, Force=V3D(), Torque=V3D(), Point=0):
        """
        Point : Position relative sur la barre [-1 (bout A), 0 (centre), 1 (bout B)]
        Force : Vecteur Force (Vector3D)
        Torque : Vecteur Couple (Vector3D), on utilise la composante Z
        """
        # Somme des forces pour la translation
        self.sum_forces += Force
        
        # Somme des moments pour la rotation
        # On suppose que Torque est un Vector3D, on prend son axe Z pour la 2D
        self.sum_moments += Torque.z
        
        # On ajoute le moment généré par la force : M = r ^ F
        # Vecteur r : du centre de masse vers le point d'application
        # r = (L/2 * Point) orienté selon l'angle theta
        if Point != 0 and Force.mod() > 0:
            dist_r = (self.length / 2.0) * Point
            # Vecteur unitaire de la barre
            u_barre = V3D(1,0,0).rotZ(self.theta)
            r = u_barre * dist_r
            
            # Produit vectoriel r ^ F
            moment_force = r * Force # Retourne un V3D
            self.sum_moments += moment_force.z
    
    def simulate(self, step):
        if not self.fixed:
            # PFD Translation : a = F / m
            self.acc = self.sum_forces * (1.0 / self.mass)
            self.speed += self.acc * step
            self.position += self.speed * step
            
            # PFD Rotation : alpha = Somme_Moments / J
            self.alpha = self.sum_moments / self.J
            self.omega += self.alpha * step
            self.theta += self.omega * step
            
        # Remise à zéro des accumulateurs
        self.sum_forces = V3D()
        self.sum_moments = 0.0
    
    def plot(self):
        # Pour les graphes statiques (matplotlib), on trace le centre
        from pylab import plot
        return plot([self.position.x], [self.position.y], 'o', color=self.color, label=self.name)
    
    def plotRot(self):
        pass
    
    def gameDraw(self, scale, screen):
        # Calcul des extrémités pour le dessin
        u_barre = V3D(1,0,0).rotZ(self.theta)
        
        demi_L = (self.length / 2.0)
        P1 = self.position + u_barre * (-demi_L)
        P2 = self.position + u_barre * (demi_L)
        
        # Conversion en pixels
        x1 = int(P1.x * scale)
        y1 = int(P1.y * scale)
        
        x2 = int(P2.x * scale)
        y2 = int(P2.y * scale)
        
        cx = int(self.position.x * scale)
        cy = int(self.position.y * scale)
        
        # Dessin de la barre
        if isinstance(self.color, str):
            c = pygame.Color(self.color)
        else:
            c = self.color
            
        pygame.draw.line(screen, c, (x1, y1), (x2, y2), 5)
        # Petit point au centre
        pygame.draw.circle(screen, (0,0,0), (cx, cy), 3)
    
    # méthode pour aider les ressorts à trouver les points d'accroche
    def getWorldPoint(self, relative_point):
        """ Retourne la position absolue d'un point relatif [-1, 1] """
        if relative_point == 0:
            return self.position
        
        u_barre = V3D(1,0,0).rotZ(self.theta)
        return self.position + u_barre * ((self.length/2.0) * relative_point)
    
    def getSpeedAtPoint(self, relative_point):
        """ Vitesse d'un point de la barre (V_G + Omega ^ GM) """
        if relative_point == 0:
            return self.speed
        
        # V = V_centre + Omega * z ^ GM
        # GM = L/2 * k * u_barre
        omega_vec = V3D(0, 0, self.omega)
        u_barre = V3D(1,0,0).rotZ(self.theta)
        GM = u_barre * ((self.length/2.0) * relative_point)
        
        v_rot = omega_vec * GM # Produit vectoriel
        return self.speed + v_rot