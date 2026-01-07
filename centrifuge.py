import pygame
from math import cos, sin
from pylab import *
from vector3D import Vector3D as V3D
from particule import Particule
from univers import Univers, Force, SpringDamper
from moteurCC import MoteurCC
from control_PID import ControlPID_vitesse

class LiaisonMoteurPhysique(Force):
    """
    Gère la contrainte cinématique et la mise à jour du moteur.
    Prend en compte un point de Pivot pour le centre de rotation.
    """
    def __init__(self, moteur, pid, particule, step, pivot=None):
        super().__init__(name="Glissière Moteur")
        self.moteur = moteur
        self.pid = pid
        self.particule = particule
        self.step = step
        self.pivot = pivot

    def setForce(self, p):
        if p == self.particule:
            
            self.pid.simule(self.step)
            
            # centre de rotation
            if self.pivot:
                center_pos = self.pivot.getPosition()
            else:
                center_pos = V3D(0,0,0)
            
            pos_rel = self.particule.getPosition() - center_pos
            r = pos_rel.mod() 
            
            theta_moteur = self.moteur.getPosition()
            omega_moteur = self.moteur.getSpeed()
            vit = self.particule.getSpeed()
            
            if r > 0:
                v_radiale = (vit ** pos_rel) / r 
            else:
                v_radiale = 0
            
            new_rel_x = r * cos(theta_moteur)
            new_rel_y = r * sin(theta_moteur)
            
            v_x = v_radiale * cos(theta_moteur) - (r * omega_moteur) * sin(theta_moteur)
            v_y = v_radiale * sin(theta_moteur) + (r * omega_moteur) * cos(theta_moteur)
            
            self.particule.position[-1] = center_pos + V3D(new_rel_x, new_rel_y, 0)
            self.particule.speed[-1]    = V3D(v_x, v_y, 0)

class UniversCentrifuge(Univers):
    """
    Une version de l'Univers qui sait dessiner le bras du moteur
    et afficher les infos de vitesse.
    """
    def __init__(self, moteur, pid, pivot=None, step=0.01, dimensions=(100,100), gameDimensions=(1024,780), game=True):
        super().__init__(name="Simulation Centrifugeuse", step=step, dimensions=dimensions, game=game, gameDimensions=gameDimensions)
        
        self.moteur = moteur
        self.pid = pid
        self.pivot = pivot
        
    def simulateRealTime(self):
        import pygame
        running = self.game
        pygame.init()
        
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
        font = pygame.font.Font('freesansbold.ttf', 18)
        
        paused = False
        
        while running:
            pygame.event.pump()
            keys = pygame.key.get_pressed()
            events = pygame.event.get()
            
            if keys[pygame.K_ESCAPE]: running = False
            for event in events:
                if event.type == pygame.QUIT: running = False
                
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_UP:
                        self.pid.target += 0.5
                        print(f"Nouvelle consigne : {self.pid.target} rad/s")
                    if event.key == pygame.K_DOWN:
                        self.pid.target -= 0.5
                        print(f"Nouvelle consigne : {self.pid.target} rad/s")
                    if event.key == pygame.K_SPACE:
                        paused = not paused
            
            if not paused:
                self.simulateFor(1/self.gameFPS)    
            
            screen.fill((250, 250, 250)) # Fond blanc
            
            # calcul du centre de rotation
            if self.pivot:
                cx = int(self.scale * self.pivot.getPosition().x)
                cy = int(self.scale * self.pivot.getPosition().y)
            else:
                cx = int(self.scale * 0)
                cy = int(self.scale * 0)
            
            # position de la particule mobile
            px = int(self.scale * self.population[0].getPosition().x)
            py = int(self.scale * self.population[0].getPosition().y)
            
            # dessin de la glissière (Ligne noire épaisse)
            pygame.draw.line(screen, (50, 50, 50), (cx, cy), (px, py), 4)
            
            # dessin du ressort (Ligne bleue fine)
            pygame.draw.line(screen, (0, 0, 255), (cx, cy), (px, py), 2)
            
            # dessin du Moteur (Cercle au centre)
            pygame.draw.circle(screen, (100, 100, 100), (cx, cy), 10)
            
            # dessin dela masse au bout
            for t in self.population:
                t.gameDraw(self.scale, screen)
            
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            # affchage du texte
            txt_vitesse = font.render(f"Omega Moteur: {self.moteur.getSpeed():.2f} rad/s", True, (0,0,0))
            txt_consigne = font.render(f"Consigne (Up/Down): {self.pid.target:.1f} rad/s", True, (0,100,0))
            txt_dist = font.render(f"Distance: {(self.population[0].getPosition()-self.pivot.getPosition()).mod():.3f} m", True, (0,0,0))
            
            screen.blit(txt_consigne, (10, 10))
            screen.blit(txt_vitesse, (10, 35))
            screen.blit(txt_dist, (10, 60))

            pygame.display.flip()
            clock.tick(self.gameFPS)
        
        pygame.quit()

def courbe_regime_permanent(k, l0, m_part):
    """
    Simule le système pour différentes vitesses de rotation et trace
    la distance d'équilibre en fonction de omega.
    """
    
    # Pour k=30, m=1, w_critique = sqrt(k/m) = sqrt(30) = 5.47 rad/s
    vitesses = arange(0, 5.5, 0.25)
    
    distances_eq = []
    omegas_eq = []
    
    step_simu = 0.005
    
    for v_target in vitesses:
        univ_simu = Univers(step=step_simu, game=False)
        
        # Moteur et PID
        motor_tmp = MoteurCC(J=0.1)
        pid_tmp = ControlPID_vitesse(motor_tmp, Kp=20.0, Ki=50.0) # PID rapide
        pid_tmp.setTarget(v_target)
        
        centre_tmp = Particule(p0=V3D(0,0,0), fix=True)
        part_tmp = Particule(mass=m_part, p0=V3D(l0, 0, 0), v0=V3D(0,0,0))
        
        ressort_tmp = SpringDamper(centre_tmp, part_tmp, k=k, c=2.0, l0=l0) # Amortissement c=2
        liaison_tmp = LiaisonMoteurPhysique(motor_tmp, pid_tmp, part_tmp, step=step_simu, pivot=centre_tmp)
        
        univ_simu.addParticule(part_tmp)
        univ_simu.addGenerators(ressort_tmp, liaison_tmp)
        
        univ_simu.simulateFor(5.0)  # 5 secondes
        
        d_final = part_tmp.getPosition().mod()
        w_final = motor_tmp.getSpeed()
        
        distances_eq.append(d_final)
        omegas_eq.append(w_final)
    
    figure("Courbe d'équilibre Centrifuge")
    plot(omegas_eq, distances_eq, 'bo-', label='Simulation Numérique')
    
    # théorique : k(d-l0) = m*w^2*d  =>  d = k*l0 / (k - m*w^2)
    w_th = arange(0, 5.4, 0.01)
    
    # On évite la division par zéro ou les valeurs négatives proches de l'asymptote
    w_th = [w for w in w_th if (k - m_part*w**2) > 0.1]
    d_th = [(k*l0)/(k - m_part*w**2) for w in w_th]
    
    plot(w_th, d_th, 'r--', label='Théorie')
    
    title(f"Distance d'équilibre vs Vitesse rotation (k={k} N/m)")
    xlabel("Vitesse de rotation (rad/s)")
    ylabel("Distance à l'axe (m)")
    grid(True)
    legend()
    show()


if __name__ == '__main__':
    
    k = 30.0
    l0 = 0.5
    m_part = 1.0
    step = 0.005
    
    courbe_regime_permanent(k, l0, m_part)
    
    W_win, H_win = 1024, 780
    largeur_monde = 3.0 
    scale = W_win / largeur_monde 
    hauteur_monde = H_win / scale
    
    OFFSET = V3D(largeur_monde / 2, hauteur_monde / 2, 0)
    
    motor = MoteurCC(name="Moteur Z", J=0.1)
    pid = ControlPID_vitesse(motor, Kp=10.0, Ki=50.0)
    pid.setTarget(2.0)
    
    centre = Particule(p0=OFFSET, fix=True, name="Pivot", color=(0,0,0))
    p = Particule(mass=m_part, p0=OFFSET + V3D(l0, 0, 0), 
                v0=V3D(0,0,0), name="Bille", color=(1, 0, 0))
    
    ressort = SpringDamper(centre, p, k=k, c=1.0, l0=l0)
    
    liaison = LiaisonMoteurPhysique(motor, pid, p, step, pivot=centre)
    
    visu = UniversCentrifuge(moteur=motor, pid=pid, pivot=centre,
                            dimensions=(largeur_monde, hauteur_monde), 
                            gameDimensions=(W_win, H_win), 
                            step=step, game=True)
    
    visu.addParticule(p)
    visu.addGenerators(ressort, liaison)
    
    visu.simulateRealTime()