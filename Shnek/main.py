from viewerGL import ViewerGL
import glutils
from mesh import Mesh
from cpe3d import Object3D, Camera, Transformation3D, Text
from fsnake import Map
import numpy as np
import OpenGL.GL as GL
import pyrr
import physics

def main():

    # Initialisation de OpenGL
    
    viewer = ViewerGL()

    viewer.set_camera(Camera())
    viewer.cam.transformation.translation.y = 2
    viewer.cam.transformation.rotation_center = viewer.cam.transformation.translation.copy()

    # Creation des Shaders

    program3d_id = glutils.create_program_from_file('shader.vert', 'shader.frag')
    programGUI_id = glutils.create_program_from_file('gui.vert', 'gui.frag')

    #Creation des Objets


    # Creation de l'objet Shrek, personnage jouable
    m = Mesh.load_obj('ressource/Shrek.obj')
    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([2, 2, 2, 1]))

    vao = m.load_to_gpu()
    nbrTriangle = m.get_nb_triangles()

    tr = physics.PhysicTransformation3D()
    tr.applyVelocity(pyrr.Vector3([0,0,20]))
    tr.applyForce(pyrr.Vector3([0,-50,0]))
    tr.translation.y = -np.amin(m.vertices, axis=0)[1]
    tr.translation.z = -5
    tr.rotation_center.z = 0.2


    texture = glutils.load_texture('ressource/Shrek_Body.png')


    o = Object3D(vao, nbrTriangle, program3d_id, texture, tr)
    viewer.add_object(o)
    viewer.shreks.append(o)

    # Passe les parametres de Shrek pour les shrek qui suivent
    viewer.setShrek(vao, nbrTriangle, program3d_id, texture)

    # pomme Shrek

    m.normalize()
    m.apply_matrix(pyrr.matrix44.create_from_scale([4, 4, 4, 2]))

    tr = Transformation3D()
    tr.translation.y = 4
    tr.translation.z = 0
    tr.rotation_center.z = 0.2


    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, tr)
    viewer.add_object(o)


    # plateau de jeu

    m = Mesh()
    p0, p1, p2, p3 = [-50, 0, -50], [50, 0, -50], [50, 0, 50], [-50, 0, 50]
    n, c = [0, 1, 0], [1, 1, 1]
    t0, t1, t2, t3 = [0, 0], [1, 0], [1, 1], [0, 1]
    m.vertices = np.array([[p0 + n + c + t0], [p1 + n + c + t1], [p2 + n + c + t2], [p3 + n + c + t3]], np.float32)
    m.faces = np.array([[0, 1, 2], [0, 2, 3]], np.uint32)
    texture = glutils.load_texture('ressource/grass.jpg')
    o = Object3D(m.load_to_gpu(), m.get_nb_triangles(), program3d_id, texture, Transformation3D())
    viewer.add_object(o)

    # Affichage du score

    vao = Text.initalize_geometry()
    texture = glutils.load_texture('ressource/fontB.jpg')
    o = Text('Score:', np.array([-1, 0.9], np.float32), np.array([-0.7, 1], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)
    o = Text(str(viewer.score), np.array([-0.7, 0.9], np.float32), np.array([-0.55, 1], np.float32), vao, 2, programGUI_id, texture)
    viewer.add_object(o)


    # Initialisation de la carte

    nbLignes = 25
    nbColonnes = 25
    carte = Map(nbLignes, nbColonnes, o, p0, p2)
    carte.mapping()
    viewer.add_map(carte)

    
    # Lancement de la boucle de jeu


    viewer.run()


if __name__ == '__main__':
    main()