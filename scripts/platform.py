import params


class Platform:

    def __init__(self, modelling, simulation, name='Platform'):
        self.modelling = modelling
        self.simulation = simulation
        self.name = name

        self.params = params.PlatformParams()
        self.__addPlatform()

    def __addPlatform(self):

        self.platform = self.modelling.addChild(self.name)
        self.simulation.addChild(self.platform)

        self.platform.addObject('MechanicalObject', position=[0], template='Vec1')
        self.platform.addObject('ArticulatedHierarchyContainer')
        self.platform.addObject('UniformMass', totalMass=self.params.mass)
        self.platform.addObject('RestShapeSpringsForceField', stiffness=1e12, points=[0])

        dim = self.params.dim

        rigid = self.platform.addChild('RigidParts')
        rigid.addObject('MechanicalObject', template='Rigid3', position=[0,0,0,0,0,0,1]*2)
        rigid.addObject('ArticulatedSystemMapping', container=self.platform.ArticulatedHierarchyContainer.getLinkPath(),
                                                    input1=self.platform.MechanicalObject.getLinkPath(),
                                                    output=rigid.MechanicalObject.getLinkPath())

        centers = self.platform.addChild('ArticulationsCenters')
        center = centers.addChild('Center')
        center.addObject('ArticulationCenter', parentIndex=0, childIndex=1, posOnParent=[dim[0]/2,0,dim[2]/2], posOnChild=[-dim[0]/2,0,-dim[2]/2], articulationProcess=0)
        articulation = center.addChild('Articulation')
        articulation.addObject('Articulation', translation=0, rotation=1, rotationAxis=[0,0,1], articulationIndex=0)


def createScene(rootnode):

    from header import addHeader, addSolvers

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation)
    rootnode.VisualStyle.displayFlags = ['showBehavior']

    Platform(modelling, simulation)
