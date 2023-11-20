def createScene(rootnode):

    from scripts.header import addHeader, addSolvers
    from scripts.robot import Robot

    settings, modelling, simulation = addHeader(rootnode, inverse=True)
    addSolvers(simulation)
    rootnode.gravity = [0, 9180, 0]

    target = modelling.addChild('Target')
    target.addObject('MechanicalObject', position=[[-5, 60, 0], [-3.5, 60, 0]], showObject=True)
    target.addObject('AnimationEditor', filename='pick-trajectory.txt', drawTimeline=False)

    box = simulation.addChild('Box')
    box.addObject('MechanicalObject', template='Rigid3', position=[-4.5, 60, 0, 0, 0, 0, 1])
    box.addObject('UniformMass', totalMass=0.001)

    colli = box.addChild('Collision')
    colli.addObject('MeshOBJLoader', filename='mesh/cube.obj', name='loader', scale3d=[0.75, 1, 0.75])
    colli.addObject('MeshTopology', src='@loader')
    colli.addObject('MechanicalObject')
    colli.addObject('TriangleCollisionModel')
    colli.addObject('LineCollisionModel')
    colli.addObject('PointCollisionModel')
    colli.addObject('RigidMapping', index=0)

    visu = box.addChild('Visu')
    visu.addObject('MeshTopology', src=colli.loader.getLinkPath())
    visu.addObject('OglModel', color=[0, 0, 1, 1])
    visu.addObject('RigidMapping', index=0)

    floor = modelling.addChild('Floor')
    floor.addObject('MechanicalObject', template='Rigid3', position=[0, 62, 0, 0, 0, 0, 1])

    colli = floor.addChild('Collision')
    colli.addObject('MeshOBJLoader', filename='mesh/cube.obj', name='loader', scale3d=[100, 0.1, 100])
    colli.addObject('MeshTopology', src='@loader')
    colli.addObject('MechanicalObject')
    colli.addObject('TriangleCollisionModel')
    colli.addObject('LineCollisionModel')
    colli.addObject('PointCollisionModel')
    colli.addObject('RigidMapping', index=0)

    visu = floor.addChild('Visu')
    visu.addObject('MeshTopology', src=colli.loader.getLinkPath())
    visu.addObject('OglModel')
    visu.addObject('RigidMapping', index=0)

    Robot(modelling, simulation, inverse=True, target=target.MechanicalObject.position)

