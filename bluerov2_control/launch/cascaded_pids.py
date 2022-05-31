from simple_launch import SimpleLauncher
from plankton_utils.time import is_sim_time
from launch.substitutions import Command

def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('gazebo',default_value=False)
    
    parameters = {'use_sim_time': sl.arg('gazebo')}
    
    with sl.group(ns=sl.arg('namespace')):        
        # load joint (camera tilt) controller only if needed
        with sl.group(if_arg='gazebo'):
            tilt_params = sl.find('bluerov2_control','tilt_control.yaml')            
            sl.node('iauv_control', 'joint_gz_pid',parameters = [tilt_params])
            
        # load body controller anyway
        sl.node('iauv_control', 'body_pid', parameters = parameters)

    return sl.launch_description()
