from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=True)
    sl.declare_arg('rviz', default_value=True)
    
    with sl.group(ns=sl.arg('namespace')):
        
        # load body controller anyway
        sl.node('auv_control', 'cascaded_pid', parameters=[sl.find('bluerov2_control', 'cascaded_pid.yaml')],
                output='screen')

        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])

            sl.node('slider_publisher', 'slider_publisher', name='tilt_control',
                    arguments=[sl.find('bluerov2_control', 'tilt.yaml')])

    with sl.group(if_arg='rviz'):
        sl.include('bluerov2_control','rviz_launch.py',
                   launch_arguments={'namespace': sl.arg('namespace'), 'use_sim_time': sl.sim_time})

    return sl.launch_description()
