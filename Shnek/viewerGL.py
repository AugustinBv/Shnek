#!/usr/bin/env python3

import OpenGL.GL as GL
import glfw
import pyrr
import numpy as np
import random
from cpe3d import Object3D, Transformation3D #,Text
import physics as p
import fsnake as f

class ViewerGL:
    def __init__(self):
        # initialisation de la librairie GLFW
        glfw.init()
        # parametrage du context OpenGL
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL.GL_TRUE)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        # creation et parametrage de la fenetre
        glfw.window_hint(glfw.RESIZABLE, False)
        self.window = glfw.create_window(800, 800, 'OpenGL', None, None)
        # parametrage de la fonction de gestion des evenements
        glfw.set_key_callback(self.window, self.key_callback)
        # activation du context OpenGL pour la fenetre
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)
        # activation de la gestion de la profondeur
        GL.glEnable(GL.GL_DEPTH_TEST)
        # choix de la couleur de fond
        GL.glClearColor(0.5, 0.6, 0.9, 1.0)
        print(f"OpenGL: {GL.glGetString(GL.GL_VERSION).decode('ascii')}")

        self.objs = []
        self.touch = {}
        self.score = 0
        self.shreks = []


    def run(self):
        # boucle d'affichage

        while not glfw.window_should_close(self.window):
            # nettoyage de la fenetre : fond et profondeur
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

            self.update_key()

            
            if p.sphereCollision(self.objs[0].transformation,self.objs[1].transformation, 3) :  # Collision avec une pomme
                self.randomPomme()
                self.update_score(1)
                self.newShrek()
       


            if self.objs[0].transformation.translation.y < 2 :
                self.objs[0].transformation.stopForce(0)
                self.objs[0].transformation.velocity.y = 0
                self.objs[0].transformation.translation.y = 2
            else:
                self.objs[0].transformation.setForce(0)

            

            for shrek in self.shreks:       # Gestion de la collision des shreks entre eux et deplacement du Shrnake
                if shrek != self.shreks[0] and p.sphereCollision(self.shreks[0].transformation,shrek.transformation, 2) : 
                    self.update_score(-self.score)
                    for shrek in self.shreks[1:]:
                        self.objs.remove(shrek)
                    self.shreks = [self.shreks[0]]
                shrek.transformation.applyPFD()
            
            self.cam.transformation.rotation_euler = self.objs[0].transformation.rotation_euler.copy() 
            self.cam.transformation.rotation_euler[pyrr.euler.index().yaw] += np.pi
            self.cam.transformation.rotation_center = self.objs[0].transformation.translation + self.objs[0].transformation.rotation_center
            self.cam.transformation.translation = self.objs[0].transformation.translation + pyrr.Vector3([0,4, 30])


            for obj in self.objs:
                GL.glUseProgram(obj.program)
                if isinstance(obj, Object3D):
                    self.update_camera(obj.program)
                obj.draw()

            # changement de buffer d'affichage pour eviter un effet de scintillement
            glfw.swap_buffers(self.window)
            # gestion des evenements
            glfw.poll_events()
        
    def key_callback(self, win, key, scancode, action, mods):
        # sortie du programme si appui sur la touche 'echappement'
        if key == glfw.KEY_ESCAPE and action == glfw.PRESS:
            glfw.set_window_should_close(win, glfw.TRUE)
        self.touch[key] = action
    
    def add_object(self, obj):
        self.objs.append(obj)

    def add_map(self, map):
        self.map = map

    def set_camera(self, cam):
        self.cam = cam

    def update_camera(self, prog):
        GL.glUseProgram(prog)
        # Recupere l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "translation_view")
        # Verifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : translation_view")
        # Modifie la variable pour le programme courant
        translation = -self.cam.transformation.translation
        GL.glUniform4f(loc, translation.x, translation.y, translation.z, 0)

        # Recupere l'identifiant de la variable pour le programme courant
        loc = GL.glGetUniformLocation(prog, "rotation_center_view")
        # Verifie que la variable existe
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_center_view")
        # Modifie la variable pour le programme courant
        rotation_center = self.cam.transformation.rotation_center
        GL.glUniform4f(loc, rotation_center.x, rotation_center.y, rotation_center.z, 0)

        rot = pyrr.matrix44.create_from_eulers(-self.cam.transformation.rotation_euler)
        loc = GL.glGetUniformLocation(prog, "rotation_view")
        if (loc == -1) :
            print("Pas de variable uniforme : rotation_view")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, rot)
    
        loc = GL.glGetUniformLocation(prog, "projection")
        if (loc == -1) :
            print("Pas de variable uniforme : projection")
        GL.glUniformMatrix4fv(loc, 1, GL.GL_FALSE, self.cam.projection)

    def update_score(self, val = 1):        # Mise à jour du score en texte sur l'ecran
        self.score += val
        self.objs[4].update_txt(str(self.score))


    def update_key(self):
        if glfw.KEY_UP in self.touch and self.touch[glfw.KEY_UP] > 0:
            self.objs[0].transformation.translation += \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.1]))
        if glfw.KEY_DOWN in self.touch and self.touch[glfw.KEY_DOWN] > 0:
            self.objs[0].transformation.translation -= \
                pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.objs[0].transformation.rotation_euler), pyrr.Vector3([0, 0, 0.1]))
        if glfw.KEY_LEFT in self.touch and self.touch[glfw.KEY_LEFT] > 0:
            self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] -= 0.05
        if glfw.KEY_RIGHT in self.touch and self.touch[glfw.KEY_RIGHT] > 0:
            self.objs[0].transformation.rotation_euler[pyrr.euler.index().yaw] += 0.05

        if glfw.KEY_SPACE in self.touch and self.touch[glfw.KEY_SPACE] > 0:
            self.objs[0].transformation.applyVelocity(pyrr.Vector3([0,2,0]))
        
        if glfw.KEY_F in self.touch and self.touch[glfw.KEY_F] > 0:
            self.randomPomme()

        if glfw.KEY_P in self.touch and self.touch[glfw.KEY_P] > 0:
            if len(self.shreks) < 50 :
                self.newShrek()
        
        if glfw.KEY_G in self.touch and self.touch[glfw.KEY_G] > 0:
            self.update_score(1)
            


    def randomPomme(self):      # Apparition aleatoire de la pomme Shrek sur un des rectangles de la carte avec une rotation aleatoire
        case = self.map.map_grid["["+str(random.randrange(self.map.lignes))+","+str(random.randrange(self.map.colonnes))+"]"]
        self.objs[1].transformation.translation = pyrr.Vector3([case.mid[0], 4 , case.mid[1]])
        self.objs[1].transformation.rotation_euler[pyrr.euler.index().yaw] += random.random()


    def setShrek(self,vao, nbrTriangle, program3d_id, texture,):    # Recuperation des informations du main pour les shreks qui suivent
        self.shrekParams = [vao, nbrTriangle, program3d_id, texture]


    def newShrek(self):     # Ajout d'un Shrek apres recuperation de la pomme

        tr = p.PhysicTransformation3D()
        tr.setTension(self.shreks[-1].transformation)
        if len(self.shreks) < 2 :
            inc = self.shreks[-1].transformation.toWorldSpace(self.shreks[-1].transformation.velocity)
            inc = (inc / inc.length) * 3
            tr.translation =  self.shreks[-1].transformation.translation - inc
        else:
            inc =  2* self.shreks[-1].transformation.translation - self.shreks[-1].transformation.translation
            tr.translation = (inc / inc.length) * 3
        self.shreks.append(Object3D(self.shrekParams[0], self.shrekParams[1], self.shrekParams[2], self.shrekParams[3], tr))
        self.add_object(self.shreks[-1])
        print(len(self.shreks))
            
