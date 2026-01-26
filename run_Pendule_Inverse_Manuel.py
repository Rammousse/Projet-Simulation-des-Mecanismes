import pygame
from types import MethodType
from univers import Univers
from pendule_Inverse import PenduleInverse


def demo_pendule_inverse_manuel():
    
    monUnivers = Univers(name="Pendule Inverse Manuel", dimensions=(20, 15), 
                        gameDimensions=(1200, 800), fps=60, step=0.001)
    
    # instanciation du système
    systeme = PenduleInverse(monUnivers)
    
    # définition du contrôle
    def myInteraction(self, events, keys):
        # Commandes ponctuelles
        for event in events:
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_b:
                    systeme.base.fix = not systeme.base.fix
                    print(f"Base mobile : {'BLOQUÉE' if systeme.base.fix else 'LIBRE'}")
                
                if event.key == pygame.K_r:
                    print("Reset")
                    systeme.reset()
                
                if event.key == pygame.K_p:
                    print("Perturbation !")
                    systeme.perturber()
        
        # Commande continuel
        FORCE_MAX = 25.0 
        if keys[pygame.K_LEFT]:
            systeme.set_commande(-FORCE_MAX)
        elif keys[pygame.K_RIGHT]:
            systeme.set_commande(FORCE_MAX)
        else:
            systeme.set_commande(0.0)
    
    # Injection et Lancement
    monUnivers.gameInteraction = MethodType(myInteraction, monUnivers)
    monUnivers.game = True
    monUnivers.simulateRealTime()

if __name__ == '__main__':
    demo_pendule_inverse_manuel()