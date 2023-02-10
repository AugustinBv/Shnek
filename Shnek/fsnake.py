from cpe3d import Object3D

class Rectangle:    # Creation d'une classe rectangle, utilisee pour le mapping. Peut servir pour beaucoup de choses dans le futur mais pas eu le temps de developper plus.
    def __init__(self, origin, sizex, sizez, state):
        self.origin = origin    # coordonnees en haut à gauche
        self.sizex = sizex      # largeur
        self.sizez = sizez      # hauteur
        self.state = state      # libre ou occupe
        self.mid = (self.origin[0] + sizex/2, self.origin[1] + sizez/2)
    
    def occupy(self):
        self.state = 1

    def free(self):
        self.state = 0

class Map:      # Class map utilisee pour la repartition des objets sur la carte, pas exploitee totalement comme la classe Rectangle
    def __init__(self, nbLignes, nbColonnes, mapMesh, extr1, extr2):    # Initialise la carte
        self.lignes = nbLignes
        self.colonnes = nbColonnes
        self.mesh = mapMesh
        self.size = self.mesh
        self.extr1 = extr1
        self.extr2 = extr2
        self.sizex = abs(extr1[0])+abs(extr1[0])
        self.sizez = abs(extr2[2])+abs(extr2[2])
        self.rectx = self.sizex/self.colonnes
        self.rectz = self.sizez/self.lignes
        self.map_grid = {}

    def mapping(self):      # Decoupe la carte en elements de classe rectangle. Sert par exemple pour le placement de la pomme Shrek
        for l in range (self.lignes) :
            for c in range (self.colonnes) :
                self.map_grid["["+str(l)+","+str(c)+"]"] = Rectangle((self.extr1[0]+l*self.rectx, self.extr1[2]+c*self.rectz), self.rectx, self.rectz, 0)







