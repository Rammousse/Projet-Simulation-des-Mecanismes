import pygame
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import odeint
from types import MethodType
from univers import Univers
from pendule_Inverse import PenduleInverse
from vector3D import Vector3D as V3D


DURATION = 5.0
PERTURBATION_FORCE = 50.0
STEP_SIMU = 0.001 # à changer en fonction de la valeur de k

class ValidationManager:
    """ Gère l'enregistrement, la synchro et le calcul analytique """
    def __init__(self, systeme):
        self.system = systeme
        self.recording = False
        self.armed = False # Pour attendre 1 frame après la perturbation
        
        self.data_t = []
        self.data_theta = []
        self.data_x = []
        self.start_state = None
        self.mode_fixe = False
        
        # Paramètres physiques extraits du système pour l'équation exacte
        self.M = systeme.base.mass
        self.m = systeme.pendule.mass
        self.L = systeme.pendule.length
        self.l = self.L / 2.0
        self.g = 9.81
        self.J_cm = systeme.pendule.J
        self.J_pivot = self.J_cm + self.m * self.l**2
        
        # Coefficients (doivent correspondre aux forces du code)
        self.c_rail = 8.0     # FrottementRail
        self.c_air = 0.05     # Viscosity (appliqué à la vitesse linéaire du centre)
        
        print("\n Commandes pour la validation : \n")
        print(" [B] Bloquer/Libérer chariot")
        print(" [P] Pichenette (Lance la mesure) \n")
        
    
    def trigger(self):
        """ Arme le déclenchement pour la prochaine frame """
        self.armed = True
        # On ne capture pas tout de suite, on attend que 'simulate' applique la force.
    
    def update(self, current_time):
        # si le déclenchement est armé on démarre l'enregistrement maintenant
        if self.armed:
            self.recording = True
            self.armed = False
            self.data_t = []
            self.data_theta = []
            self.data_x = []
            self.mode_fixe = self.system.base.fix
            print(f"Capture démarrée pour le mode {'FIXE' if self.mode_fixe else 'LIBRE'}")
        
        # s i l'enregistrement est en cours, on stocke
        if self.recording:
            self.data_t.append(current_time)
            self.data_theta.append(self.system.pendule.theta)
            self.data_x.append(self.system.base.getPosition().x)
            
            # Fin de l'enregistrement
            if (self.data_t[-1] - self.data_t[0]) >= DURATION:
                self.recording = False
                self.plot_comparison()
    
    def equations_fixe(self, y, t):
        """ Pendule simple amorti """
        theta, omega = y
        
        # Couple Gravité (theta=pi/2 est haut, donc cos(theta) pour le moment)
        tau_g = -self.m * self.g * self.l * np.cos(theta)
        
        # Couple Frottement Visqueux
        # La force visqueuse F = -c_air * v_centre = -c_air * (l * omega)
        # Couple = l * F = -c_air * l^2 * omega
        tau_f = -self.c_air * (self.l**2) * omega
        
        acc_ang = (tau_g + tau_f) / self.J_pivot
        return [omega, acc_ang]
    
    def equations_libre(self, y, t):
        """ Lagrange chariot + pendule """
        x, v, theta, omega = y
        
        sin_t = np.sin(theta)
        cos_t = np.cos(theta)
        
        # matrice de Masse
        a11 = self.M + self.m
        a12 = -self.m * self.l * sin_t
        a21 = a12
        a22 = self.J_cm + self.m * (self.l**2)
        
        # Vecteur Forces/Coriolis
        # Eq X: ... = F_ext - frottement_rail + m*l*cos*omega^2
        # F_ext = 0 ici
        b1 = -self.c_rail * v + self.m * self.l * cos_t * (omega**2)
        
        # Eq Theta: ... = Couple_ext - frottement_air - m*g*l*cos
        b2 = -(self.c_air * self.l**2) * omega - self.m * self.g * self.l * cos_t
        
        # Cramer ou inversion 2x2
        det = a11 * a22 - a12 * a21
        acc_x = (b1 * a22 - b2 * a12) / det
        acc_theta = (a11 * b2 - a21 * b1) / det
        
        return [v, acc_x, omega, acc_theta]
    
    def plot_comparison(self):
        t_arr = np.array(self.data_t)
        t_rel = t_arr - t_arr[0]
        
        # Conditions initiales prises au début de la capture
        theta0 = self.data_theta[0]
        x0 = self.data_x[0]
        
        # Estimation des vitesses initiales
        # On utilise les 2 premiers points pour estimer la vitesse instantanée
        if len(self.data_t) > 2:
            dt = t_rel[1] - t_rel[0]
            v0 = (self.data_x[1] - self.data_x[0]) / dt
            omega0 = (self.data_theta[1] - self.data_theta[0]) / dt
        else:
            v0, omega0 = 0, 0
        
        # résolution
        if self.mode_fixe:
            sol = odeint(self.equations_fixe, [theta0, omega0], t_rel)
            theta_ana = sol[:, 0]
            x_ana = np.full_like(t_rel, x0)
            titre = "Mode: CHARIOT BLOQUÉ"
        else:
            sol = odeint(self.equations_libre, [x0, v0, theta0, omega0], t_rel)
            x_ana = sol[:, 0]
            theta_ana = sol[:, 2]
            titre = "Mode: CHARIOT LIBRE"
        
        plt.figure(figsize=(10, 8))
        
        plt.subplot(211)
        plt.title(f"Validation Analytique vs Simulation ({titre})")
        plt.plot(t_rel, self.data_theta, 'r.', label='Simulation', ms=4)
        plt.plot(t_rel, theta_ana, 'k-', label='Analytique', lw=1.5)
        plt.ylabel('Angle (rad)')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(212)
        plt.plot(t_rel, self.data_x, 'b.', label='Simulation X', ms=4)
        plt.plot(t_rel, x_ana, 'k-', label='Analytique X', lw=1.5)
        plt.ylabel('Position X (m)')
        plt.xlabel('Temps (s)')
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()

def demo_validation():
    monUnivers = Univers(name="Validation Pendule", dimensions=(20, 15), 
                        gameDimensions=(1200, 800), fps=60, step=STEP_SIMU)
    
    systeme = PenduleInverse(monUnivers)
    validator = ValidationManager(systeme)
    
    def interaction(self_univ, events, keys):
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_b:
                    systeme.base.fix = not systeme.base.fix
                    print(f"Base mobile : {'BLOQUÉE' if systeme.base.fix else 'LIBRE'}")
                
                if event.key == pygame.K_r:
                    systeme.reset()
                    validator.recording = False
                
                if event.key == pygame.K_p:
                    print("Perturbation !")
                    systeme.perturber(PERTURBATION_FORCE)
                    # On demande au validateur de commencer la mesure à la prochaine frame
                    # pour inclure l'effet de la force dans la vitesse initiale
                    validator.trigger()
        
        # mise à jour du recorder
        validator.update(self_univ.time[-1])
    
    monUnivers.gameInteraction = MethodType(interaction, monUnivers)
    monUnivers.game = True
    monUnivers.simulateRealTime()

if __name__ == '__main__':
    demo_validation()