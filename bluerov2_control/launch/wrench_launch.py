from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=True)
    sl.declare_arg('rviz', default_value=True)
    
    with sl.group(ns=sl.arg('namespace')):

        sl.node('slider_publisher', 'slider_publisher', name='wrench_control',
                arguments=[sl.find('auv_control', 'wrench.yaml')])

        # load body controller anyway
        sl.node('thruster_manager', 'thruster_manager_node', parameters=[sl.find('bluerov2_control', 'thruster_manager.yaml')])
                
    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control','rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()
