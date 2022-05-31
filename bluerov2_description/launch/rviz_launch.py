from simple_launch import SimpleLauncher, IgnitionBridge
from plankton_utils.time import is_sim_time
from launch.substitutions import Command

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = False)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    
    sl.node('rviz2', arguments = ['-d', sl.find('bluerov2_description', 'bluerov2.rviz')])
        
    with sl.group(ns=sl.arg('namespace')):        
        sl.node('bluerov2_description', 'rviz_bridge.py')
    
    return sl.launch_description()
