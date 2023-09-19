def createScene(rootnode):

    from scripts.header import addHeader, addSolvers
    from scripts.robot import Robot

    settings, modelling, simulation = addHeader(rootnode)
    addSolvers(simulation)
    rootnode.VisualStyle.displayFlags = ['showBehavior']

    robot = Robot(modelling, simulation)
    # Uncomment this line to enable ROS communication
    # from controller import SofaROSInterface
    # rootnode.addObject(SofaROSInterface(robot))
