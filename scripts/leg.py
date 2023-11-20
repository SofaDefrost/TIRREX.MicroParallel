import params
from splib3.numerics import Vec3, Quat
from math import pi
import numpy as np


class Leg:

    def __init__(self, structure, name='Leg', index=0, translation=[0, 0, 0], rotation=[0, 0, 0]):
        self.structure = structure
        self.name = name
        self.translation = translation
        self.rotation = rotation
        self.index = index

        self.params = params.LegParams()
        self.__addLeg()
        self.__addVisu()

    def __addLeg(self):
        positions = list(np.copy(self.structure.dofs.position.value))
        index = len(positions)
        t = self.translation
        r = self.rotation

        v = Vec3([1., 0., 0.]).rotateFromEuler(np.copy(r)).scale(self.params.width)
        q = Quat([0., 0., 0., 1.]).rotateFromEuler(np.copy(r))
        DOF0TransformNode0 = list(t) + list(q)
        pos = [v[0]+t[0], v[1]+t[1], v[2]+t[2]] + list(q.rotateFromQuat(Quat([0., 0., -0.707, 0.707])))
        positions += [pos]
        self.structure.dofs.position.value = positions

        self.beam = self.structure.addChild(self.name)
        self.beam.addObject('EdgeSetTopologyContainer', edges=[self.index, index])
        self.beam.addObject('AdaptiveBeamForceFieldAndMass', massDensity=self.params.density)
        self.beam.addObject('BeamInterpolation', straight=False, dofsAndBeamsAligned=False, radius=self.params.radius,
                            DOF0TransformNode0=DOF0TransformNode0,
                            defaultYoungModulus=self.params.youngModulus, defaultPoissonRatio=self.params.poissonRatio)

    def __addVisu(self):
        visu = self.beam.addChild("Visu")
        visu.addObject("MeshOBJLoader", filename="mesh/cylinder.obj",
                       scale3d=[1, 0.0999, 1],
                       translation=[0.00001, 0, 0],
                       rotation=[0, 0, -90])
        visu.addObject("OglModel", src=visu.MeshOBJLoader.linkpath)
        visu.addObject('AdaptiveBeamMapping', useCurvAbs=False)


def createScene(rootnode):
    from scripts.header import addHeader, addSolvers
    import params

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation)
    rootnode.VisualStyle.displayFlags = ['showBehavior']

    params = params.LegParams()
    structure = modelling.addChild('Structure')
    simulation.addChild(structure)
    structure.addObject('MechanicalObject', template='Rigid3', name='dofs',
                        position=[0., params.height, 0., 0., 0., 0., 1.])

    nbLegs = 4
    r = 2. * pi / nbLegs
    for i in range(nbLegs):
        q = Quat.createFromAxisAngle([0., 1., 0.], r * i)
        translation = list(Vec3([3., 0., 0.]).rotateFromQuat(q))
        Leg(structure, name='Leg' + str(i), rotation=[0., r * i, 0.], translation=translation)

    structure.addObject('RestShapeSpringsForceField', points=[0])
