import params
from splib3.numerics import Vec3, Quat
from math import pi
from scripts.leg import Leg

import os
path = os.path.dirname(os.path.abspath(__file__))


class Robot:
    
    def __init__(self, modelling, simulation, name='Robot', inverse=False, target=None):
        self.modelling = modelling
        self.simulation = simulation
        self.name = name
        self.inverse = inverse
        self.target = target

        self.params = params.RobotParams()
        self.__addRobot()
        self.__addVisu()
        self.__addCollision()

    def __addRobot(self):

        self.node = self.modelling.addChild(self.name)
        self.anchor = self.modelling.addChild('Anchor')
        self.simulation.addChild(self.node)

        self.node.addObject('MechanicalObject', template='Rigid3', name='dofs',
                             position=[[-self.params.platform.gapX, self.params.leg.height, 0., 0., 0., 0., 1.],
                                       [self.params.platform.gapX, self.params.leg.height, 0., 0., 0., 0., 1.]])

        self.platform = self.node.addChild('Platform')
        self.platform.addObject('EdgeSetTopologyContainer', edges=[0, 1])
        self.platform.addObject('AdaptiveBeamForceFieldAndMass', massDensity=self.params.platform.density)
        self.platform.addObject('BeamInterpolation', dofsAndBeamsAligned=False, radius=self.params.platform.radius,
                                    defaultYoungModulus=self.params.platform.youngModulus,
                                    defaultPoissonRatio=self.params.platform.poissonRatio)

        if self.inverse:
            effector = self.node.addChild('Effector')
            effector.addObject('MechanicalObject', position=[[6, 16, 0], [-6, 16, 0]])
            effector.addObject('PositionEffector', indices=[0, 1],
                               effectorGoal=self.target.getLinkPath() if self.target is not None else [[0, 0, 0]]*2)
            effector.addObject('RigidMapping', rigidIndexPerPoint=[0, 1])

        nbLegs = 4
        r = 2. * pi / nbLegs
        for i in range(nbLegs):
            q = Quat.createFromAxisAngle([0., 1., 0.], r * i + 3 * pi / 4)
            translation = list(Vec3([7., 0., 0.]).rotateFromQuat(q))
            Leg(self.node, name='Leg' + str(i), index=[0, 0, 1, 1][i],
                        rotation=[0., r * i + 3 * pi / 4, 0.],
                        translation=translation)

        # Constraints
        self.anchor.addObject('MechanicalObject', template='Rigid3', name='dofs',
                                 position=self.node.dofs.position.value[2:nbLegs+2])
        self.node.addObject('PartialFixedConstraint', indices=list(range(2, nbLegs+2)),
                             fixedDirections=[1, 0, 1, 1, 1, 1])
        if self.inverse:
            for i in range(nbLegs):
                self.node.addObject('SlidingActuator', name='sa'+str(i), template='Rigid3', direction=[0, 1, 0, 0, 0, 0],
                                    maxDispVariation=0.1, indices=i)
        else:
            self.node.addObject('RestShapeSpringsForceField', points=list(range(2, nbLegs+2)),
                                external_rest_shape=self.anchor.dofs.getLinkPath(),
                                external_points=list(range(4)))

    def __addVisu(self):
        visuLeft = self.node.addChild('VisuLeft')
        visuLeft.addObject('MeshOBJLoader', filename=path+'/../mesh/gripperLeft.obj', name='loader')
        visuLeft.addObject('OglModel', src='@loader', rotation=[-90, 180, 0])
        visuLeft.addObject('RigidMapping', index=0)

        visuRight = self.node.addChild('VisuRight')
        visuRight.addObject('MeshOBJLoader', filename=path+'/../mesh/gripperRight.obj', name='loader')
        visuRight.addObject('OglModel', src='@loader', rotation=[-90, 180, 0])
        visuRight.addObject('RigidMapping', index=1)

    def __addCollision(self):
        colliLeft = self.node.addChild('CollisionLeft')
        colliLeft.addObject('MeshOBJLoader', filename=path+'/../mesh/gripperLeft.obj', name='loader', rotation=[-90, 180, 0])
        colliLeft.addObject('MeshTopology', src='@loader')
        colliLeft.addObject('MechanicalObject')
        colliLeft.addObject('TriangleCollisionModel')
        colliLeft.addObject('LineCollisionModel')
        colliLeft.addObject('PointCollisionModel')
        colliLeft.addObject('RigidMapping', index=0)

        colliRight = self.node.addChild('CollisionRight')
        colliRight.addObject('MeshOBJLoader', filename=path+'/../mesh/gripperRight.obj', name='loader', rotation=[-90, 180, 0])
        colliRight.addObject('MeshTopology', src='@loader')
        colliRight.addObject('MechanicalObject')
        colliRight.addObject('TriangleCollisionModel')
        colliRight.addObject('LineCollisionModel')
        colliRight.addObject('PointCollisionModel')
        colliRight.addObject('RigidMapping', index=1)


def createScene(rootnode):

    from scripts.header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation)
    rootnode.VisualStyle.displayFlags = ['showBehavior']

    Robot(modelling, simulation)
