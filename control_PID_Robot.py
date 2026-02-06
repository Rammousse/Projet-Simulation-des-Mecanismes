from math import pi

class ControlPIDRobot:
    """ PID avec Anti-Windup et gestion du passage à PI """
    def __init__(self, Kp, Ki, Kd, output_limit=24.0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.target = 0
        self.integral = 0
        self.prev_error = 0
        self.output_limit = output_limit
    
    def compute(self, measurement, dt):
        # calcul de l'erreur brute
        error = self.target - measurement
        
        # On normalise l'erreur pour qu'elle soit toujours entre -pi et pi
        # Cela force le robot à prendre le chemin le plus court
        error = (error + pi) % (2 * pi) - pi
        
        # terme Proportionnel
        p_term = self.Kp * error
        
        # terme Dérivé
        derivative = (error - self.prev_error) / dt
        d_term = self.Kd * derivative
        
        # terme Intégral avec Anti-Windup
        self.integral += error * dt
        limit_int = self.output_limit / (self.Ki if self.Ki > 0 else 1.0)
        self.integral = max(-limit_int, min(limit_int, self.integral))
        
        i_term = self.Ki * self.integral
        
        self.prev_error = error
        
        output = p_term + i_term + d_term
        return output