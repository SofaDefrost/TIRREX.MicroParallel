
def addHeader(rootnode, multithreading=False, inverse=False):
    """
    Adds to rootnode the default headers for a simulation with contact. Also adds and returns three nodes:
        - Settings
        - Modelling
        - Simulation

    Args:
        rootnode:
        multithreading:

    Usage:
        addHeader(rootnode)

    Returns:
        the three SOFA nodes {settings, modelling, simulation}
    """
    settings = rootnode.addChild('Settings')
    settings.addObject('BackgroundSetting', color=[1, 1, 1, 1])
    settings.addObject('RequiredPlugin', name='Plugins', pluginName=['Sofa.Component.SceneUtility',
                                                                     'Sofa.Component.Visual',
                                                                     'BeamAdapter'])
    settings.addObject('AttachBodyButtonSetting', stiffness=1)

    rootnode.addObject('VisualStyle')
    rootnode.addObject('CollisionPipeline')
    rootnode.addObject("DefaultVisualManagerLoop")
    rootnode.addObject('RuleBasedContactManager', responseParams='mu=0.0', response='FrictionContactConstraint')
    rootnode.addObject('BruteForceBroadPhase')
    rootnode.addObject('BVHNarrowPhase')
    rootnode.addObject('LocalMinDistance', alarmDistance=2, contactDistance=0.1)
    rootnode.addObject('FreeMotionAnimationLoop', parallelCollisionDetectionAndFreeMotion=multithreading,
                       parallelODESolving=multithreading)
    if inverse:
        settings.addObject('RequiredPlugin', name='SoftRobots.Inverse')
        rootnode.addObject('QPInverseProblemSolver', name='ConstraintSolver', tolerance=1e-8, maxIterations=100,
                           multithreading=multithreading)
    else:
        rootnode.addObject('GenericConstraintSolver', name='ConstraintSolver', tolerance=1e-8, maxIterations=100,
                           multithreading=multithreading)
    rootnode.gravity = [0, -9810, 0]
    rootnode.dt = 0.01

    modelling = rootnode.addChild('Modelling')
    simulation = rootnode.addChild('Simulation')
    return settings, modelling, simulation


def addSolvers(node, template='CompressedRowSparseMatrixd', rayleighMass=0.01, rayleighStiffness=0.01, firstOrder=False, cuda=False):
    """
    Adds solvers (EulerImplicitSolver, LDLSolver, GenericConstraintCorrection) to the given node.

    Args:
        node:
        template: for the LDLSolver
        rayleighMass:
        rayleighStiffness:
        firstOrder: for the implicit scheme
        cuda: requires the plugin SofaCUDASolvers

    Usage:
        ddSolvers(node)
    """
    node.addObject('RequiredPlugin', name='SofaPlugins', pluginName=['Sofa.Component.LinearSolver.Direct',
                                                                     'Sofa.Component.ODESolver.Backward'])
    node.addObject('EulerImplicitSolver', firstOrder=firstOrder, rayleighStiffness=rayleighStiffness,
                   rayleighMass=rayleighMass)
    if cuda:
        node.addObject('RequiredPlugin', name='SofaCUDASolvers')
        node.addObject('CudaSparseLDLSolver', name='Solver', template='AsyncCompressedRowSparseMatrixMat3x3f',
                       useMultiThread=True)
    else:
        node.addObject('SparseLDLSolver', name='Solver', template=template)
    node.addObject('GenericConstraintCorrection', linearSolver=node.Solver.getLinkPath())


# Test
def createScene(rootnode):

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation)
