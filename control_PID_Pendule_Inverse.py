import math

class ControlPID_Pendule():
    """
    Contrôleur par retour d'état pour le Pendule Inversé.
    """
    def __init__(self, x_target=10.0, theta_target=math.pi/2):
        self.x_target = x_target
        self.theta_target = theta_target
        
        # Gains pour l'angle
        self.Kp_theta = 4000.0  # Raideur angulaire (Réaction proportionnelle à l'erreur d'angle)
        self.Kd_theta = 600.0   # Amortissement angulaire (Freine la rotation)
        
        # Gains pour la position (revenir au centre)
        self.Kp_x = 30.0        # Rappel vers la position x_target
        self.Kd_x = 60.0        # Amortissement du chariot (Freine la vitesse linéaire)
        
    def compute(self, theta, omega, x, v):
        """
        Calcule la force à appliquer au chariot.
        :param theta: Angle actuel du pendule (radians)
        :param omega: Vitesse angulaire du pendule (rad/s)
        :param x: Position x du chariot (m)
        :param v: Vitesse x du chariot (m/s)
        :return: Force (Newton)
        """
        
        # Erreur angulaire
        error_theta = self.theta_target - theta
        
        # Erreur de position
        error_x = self.x_target - x
        
        # Loi de commande (Somme pondérée)
        force_angle = (self.Kp_theta * error_theta) - (self.Kd_theta * omega)
        
        # Pour la position, c'est l'inverse, si on est trop à droite (x > target),
        # on veut une force à gauche (négative).
        force_pos   = (self.Kp_x * error_x) - (self.Kd_x * v)
        
        total_force = force_angle + force_pos
        
        # Saturation pour éviter des forces irréalistes
        MAX_FORCE = 3000.0
        total_force = max(-MAX_FORCE, min(MAX_FORCE, total_force))
        
        return total_force