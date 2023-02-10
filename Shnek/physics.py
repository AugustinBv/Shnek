
from cmath import sqrt
from cpe3d import Transformation3D
import pyrr, glfw
import cpe3d
import numpy as np

def pointRectangleCollision( target = Transformation3D(), rectangle = [pyrr.Vector3(), 1, 1] ):     # Collision entre un point et un rectangle de la carte, peut etre utilise pour le placement d'objets
    test = False
    pointX, pointY = target.translation.x, target.translation.z
    rectX, rectY = rectangle[0].x, rectangle[0].z
    if pointX >= rectX and pointX <= rectX + rectangle[1] and pointY >= rectY and pointY <= rectY + rectangle[2] : test = True
    return test

def sphereCollision( target = Transformation3D(), center = Transformation3D(), padding = 10.0 ) :   # Collision spherique entre les Shreks ainsi que la pomme

    return((center.translation - target.translation).length < padding)

class PhysicTransformation3D(cpe3d.Transformation3D) : 

    def __init__(self, euler = pyrr.euler.create(), center = pyrr.Vector3(), translation = pyrr.Vector3(), velocity = pyrr.Vector3(), force = pyrr.Vector3() ):
        super().__init__(euler, center, translation)
        self.velocity = pyrr.Vector3()
        self.force = []
        self.forceApplies = []
        self.lastTime= 0
        self.tension = False
        self.tensionScale = 500

    def toWorldSpace(self, vector = pyrr.Vector3()): 

        x,y,z = pyrr.matrix33.apply_to_vector(pyrr.matrix33.create_from_eulers(self.rotation_euler), vector)
        return pyrr.Vector3([x,y,z])

    def getDeltaTime(self):
        if self.lastTime == 0:
            self.lastTime = glfw.get_time()
        delta = glfw.get_time() - self.lastTime
        self.lastTime = glfw.get_time()
        return delta

    def applyForce(self, force = pyrr.Vector3()):
        self.force.append(force)
        self.forceApplies.append(1)
    
    def stopForce(self, indForce = 0):
        self.forceApplies[indForce] = 0
    
    def setForce(self, indForce = 0):
        self.forceApplies[indForce] = 1

    def applyVelocity(self, velocity = pyrr.Vector3()):
        self.velocity += velocity

    def applyPFD(self):
        delta = self.getDeltaTime()   
        acceleration = pyrr.Vector3()
        for i  in range(len(self.force)) :
            acceleration += self.force[i] * self.forceApplies[i] 
        
        tensionForce = pyrr.Vector3()
        if self.tension :
            dir = (self.tensionTarget.translation - self.translation)
            dir = (dir / dir.length) * (dir.length - 1)
            tensionForce = dir * self.tensionScale

            ref = self.toWorldSpace(pyrr.Vector3([0,0,1]))
            angle = np.arccos((pyrr.vector.dot(ref, dir))/ (dir.length * ref.length))
            self.rotation_euler[pyrr.euler.index().yaw] += angle *delta *delta * self.tensionScale

        self.velocity += acceleration * delta
        self.translation = self.translation + self.toWorldSpace(self.velocity * delta) + tensionForce * delta * delta


    def setTension(self, other):
        self.tensionTarget = other
        self.tension = True

    



