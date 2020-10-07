from controller import Robot, Supervisor, Node, Field
def simulation_reset(node, field, initial_translation, initial_rotation):
    for key in field.keys():
        if "translation" in key:
            field[key].setSFVec3f(initial_translation[key])
        if "rotation" in key:
            field[key].setSFRotation(initial_rotation[key])
    for key in node.keys():
        if "e-puck" in key:
            node[key].restartController()
