from math import pi
from vector3D import Vector3D as V3D
from univers import Gravity, LiaisonPivot, LiaisonGlissiere, Viscosity, Force
from particule import Particule
from barre2D import Barre2D


class MoteurLineaire(Force):
    """ Actuateur : Pousse le chariot selon l'axe X. """
    def __init__(self, particule_cible):
        super().__init__(name="Moteur Lineaire")
        self.cible = particule_cible
        self.commande = 0.0 # Force en Newtons
        
    def setForce(self, p):
        if p == self.cible:
            p.applyForce(V3D(self.commande, 0, 0))

class FrottementRail(Force):
    """ Résistance : Simule le frottement des roues sur le rail. """
    def __init__(self, particule_cible, coeff=5.0):
        super().__init__(name="Frottement Rail")
        self.cible = particule_cible
        self.coeff = coeff
        
    def setForce(self, p):
        if p == self.cible:
            vx = p.getSpeed().x
            p.applyForce(V3D(-self.coeff * vx, 0, 0))


class PenduleInverse:
    def __init__(self, univers, x_init=10.0, y_rail=5.0, l_pendule=3.0, m_chariot=2.0, m_pendule=1.0):
        """
        Construit le système physique complet et l'ajoute à l'univers donné.
        """
        self.univers = univers
        
        # paramètres géométriques
        self.X_INIT = x_init
        self.Y_RAIL = y_rail
        self.L_PENDULE = l_pendule
        
        # Base Mobile (Chariot)
        self.base = Particule(mass=m_chariot, p0=V3D(x_init, y_rail), v0=V3D(), 
                            color='blue', name="Base Mobile")
        
        # Pendule (Barre) initialisé dans pos vertical haut (pi/2)
        pos_centre_pendule = V3D(x_init, y_rail + l_pendule/2.0)
        self.pendule = Barre2D(mass=m_pendule, long=l_pendule, theta=pi/2, 
                            pos=pos_centre_pendule, color='red', nom="Pendule")
        
        # création des Liaisons et Forces on peut mettre k = 50000 mais donc step = 0.0001
        self.glissiere = LiaisonGlissiere(self.base, y_fixe=y_rail, k=4000, c=150)
        self.pivot = LiaisonPivot(self.base, self.pendule, anchor1=-1, k=4000, c=100)
        self.g = Gravity(V3D(0, -9.81))
        
        # actuateurs
        self.moteur = MoteurLineaire(self.base)
        self.frottement = FrottementRail(self.base, coeff=8.0)
        self.air = Viscosity(coeff=0.05)
        
        univers.addParticule(self.base, self.pendule)
        univers.addGenerators(self.g, self.glissiere, self.pivot, 
                            self.moteur, self.frottement, self.air)
    
    def reset(self):
        """ Remet le système à l'état initial c-a-d vertical haut, vitesse nulle. """
        self.base.fix = False
        self.base.position[-1] = V3D(self.X_INIT, self.Y_RAIL)
        self.base.speed[-1] = V3D()
        self.base.forces = V3D()
        
        self.pendule.position = V3D(self.X_INIT, self.Y_RAIL + self.L_PENDULE/2.0)
        self.pendule.speed = V3D()
        self.pendule.theta = pi/2
        self.pendule.omega = 0
        self.pendule.sum_forces = V3D()
        self.pendule.sum_moments = 0.0
        
        self.moteur.commande = 0.0
    
    def set_commande(self, force_newton):
        """ Interface simplifiée pour le contrôleur """
        self.moteur.commande = force_newton
    
    def perturber(self, force_x=50):
        """ Applique une pichenette au sommet du pendule """
        self.pendule.applyEffort(Force=V3D(force_x,0,0), Point=1)