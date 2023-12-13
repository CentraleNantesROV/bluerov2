from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('camera', False)
    sl.declare_arg('jsp', False)
    
    namespace = sl.arg('namespace')
    
    with sl.group(ns=namespace):

        sl.robot_state_publisher('bluerov2_ignition', 'bluerov2.urdf')

        with sl.group(if_arg='jsp'):
            sl.joint_state_publisher(True)
        
    return sl.launch_description()
