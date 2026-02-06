import pygame
from types import MethodType
from vector3D import Vector3D as V3D
from univers import Univers, Pichenette, Force
from pendule_Inverse import PenduleInverse
from control_PID_Pendule_Inverse import ControlPID_Pendule
from moteurCC import MoteurCC


class ActuateurMoteurCC(Force):
    """
    Remplace le MoteurLineaire.
    Convertit la tension électrique (U) en Force mécanique sur le chariot (F = ma).
    Modèle : Roue de rayon r roulant sur le rail sans glissement.
    """
    def __init__(self, target_particule, moteur_cc, rayon_roue=0.05):
        super().__init__(name="Actuateur CC")
        self.cible = target_particule
        self.motor = moteur_cc
        self.r = rayon_roue  # Rayon de l'axe de transmission (m)
        self.tension_commande = 0.0 # Tension appliquée (Volts)

    def set_voltage(self, voltage):
        """ Applique la tension bornée par le moteur """
        self.tension_commande = max(-self.motor.Vmax, min(self.motor.Vmax, voltage))
        self.motor.setVoltage(self.tension_commande)

    def setForce(self, p):
        # On n'applique la force que si on traite la particule cible (le chariot)
        if p != self.cible:
            return

        # récupération de la cinématique actuelle
        v_chariot = p.getSpeed().x  # Vitesse linéaire (m/s)
        
        # conversion en cinématique angulaire moteur
        # omega = v / r
        omega_moteur = v_chariot / self.r
        
        # On met à jour l'état interne du moteur (pour ses graphes/logs)
        # Note: On force la vitesse du moteur car il est mécaniquement lié au rail
        self.motor.omega.append(omega_moteur) 
        
        # calcul du Courant (Loi d'Ohm généralisée : U = E + RI)
        # E = ke * omega
        e = self.motor.ke * omega_moteur
        
        # I = (U - E) / R  (On néglige l'inductance L pour la dynamique rapide ici)
        i = (self.tension_commande - e) / self.motor.R
        
        # calcul du Couple Moteur
        # Gamma = kc * I
        couple = self.motor.kc * i
        
        # conversion Couple -> Force Linéaire
        # F = Gamma / r
        force_traction = couple / self.r
        
        # Application de la force au chariot
        p.applyForce(V3D(force_traction, 0, 0))


def demo_pendule_inverse_moteur_cc():
    monUnivers = Univers(name="Pendule Inverse - Moteur CC", dimensions=(20, 15), 
                        gameDimensions=(1200, 800), fps=60, step=0.001)
    systeme = PenduleInverse(monUnivers)

    # on retire l'ancien moteur linéaire de la liste des générateurs de l'univers
    if systeme.moteur in monUnivers.generators:
        monUnivers.generators.remove(systeme.moteur)
    
    # création du Moteur CC (On choisit un moteur assez coupleux pour le pendule)
    # R=1.5 Ohm, ke=0.1, kc=0.1 (moteur standard type Maxon simulé)
    mon_moteur = MoteurCC(name="Moteur Propulsion", R=1.5, ke=0.5, kc=0.5, Vmax=24.0)
    
    # Création de l'interface Actuateur (Rayon de transmission r=50cm)
    rayon_roue = 0.1
    actuateur_cc = ActuateurMoteurCC(systeme.base, mon_moteur, rayon_roue=rayon_roue)
    
    # ajout au système et à l'univers
    systeme.moteur_cc = actuateur_cc # On le stocke dans le système
    monUnivers.addGenerators(actuateur_cc)

    def draw_sans_vecteur(self, scale, screen):
        X = int(scale * self.getPosition().x)
        Y = int(scale * self.getPosition().y)
        size = 3
        c = (0, 0, 255) # Bleu
        pygame.draw.circle(screen, c, (X, Y), size*2, size)

    systeme.base.gameDraw = MethodType(draw_sans_vecteur, systeme.base)
    
    controller = ControlPID_Pendule(x_target=10.0, theta_target=3.14159/2, Kp_theta= 150000, Kd_theta =10000, Kp_x = 30.0, Kd_x = 60.0)

    def myInteraction(self, events, keys):
        
        # Gestion des événements (Pichenettes & Reset)
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_RIGHT:
                    print(">>> Perturbation Droite")
                    p = Pichenette(target=systeme.pendule, force_vector=V3D(8, 0, 0), duration=0.1, step=self.step)
                    self.addGenerators(p)
                
                elif event.key == pygame.K_LEFT:
                    print("<<< Perturbation Gauche")
                    p = Pichenette(target=systeme.pendule, force_vector=V3D(-8, 0, 0), duration=0.1, step=self.step)
                    self.addGenerators(p)
                    
                elif event.key == pygame.K_r:
                    print("Reset")
                    systeme.reset()
                    systeme.moteur_cc.set_voltage(0) # Reset tension
                    # Nettoyage des pichenettes
                    self.generators = [g for g in self.generators if not isinstance(g, Pichenette)]
        
        # récupération de l'état (Capteurs)
        theta = systeme.pendule.theta
        omega = systeme.pendule.omega
        x_chariot = systeme.base.getPosition().x
        v_chariot = systeme.base.getSpeed().x
        
        # PID : Calcul de la FORCE désirée (en Newtons)
        force_consigne = controller.compute(theta, omega, x_chariot, v_chariot)
        
        # modèle Inverse : Calcul de la TENSION nécessaire (en Volts)
        # On inverse l'équation du moteur : F = (kc/r) * (U - ke*w) / R
        # Donc : U = (F * R * r / kc) + (ke * w)
        # Avec w = v / r
        
        R = mon_moteur.R
        kc = mon_moteur.kc
        ke = mon_moteur.ke
        r = actuateur_cc.r
        
        # Terme de puissance (pour créer la force)
        u_force = (force_consigne * R * r) / kc
        
        # Terme de compensation de la FCEM (pour contrer la vitesse)
        omega_moteur = v_chariot / r
        u_fem = ke * omega_moteur
        
        # Tension totale théorique
        tension_target = u_force + u_fem
        
        # application de la tension (l'actuateur gère la saturation +/- 24V)
        systeme.moteur_cc.set_voltage(tension_target)

    # Injection de la méthode d'interaction
    monUnivers.gameInteraction = MethodType(myInteraction, monUnivers)
    
    # Lancement
    monUnivers.game = True
    monUnivers.simulateRealTime()

if __name__ == '__main__':
    print("\n PENDULE INVERSE AVEC MOTEUR CC \n")
    print(" Le PID calcule une force, convertie en tension pour le moteur.")
    print(" [Flèches] : Perturbations")
    print(" [R] : Reset")
    
    demo_pendule_inverse_moteur_cc()