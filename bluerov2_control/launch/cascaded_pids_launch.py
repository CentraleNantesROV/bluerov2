from simple_launch import SimpleLauncher

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('sliders', default_value=False)
    
    with sl.group(ns=sl.arg('namespace')):
        
        # load body controller anyway
        sl.node('auv_control', 'cascaded_pid', parameters=[sl.find('bluerov2_control', 'cascaded_pid.yaml')])
                
        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', 'slider_publisher', name='pose_control',
                    arguments=[sl.find('auv_control', 'pose_setpoint.yaml')])
            
            sl.node('slider_publisher', 'slider_publisher', name='tilt_control',
                    arguments=[sl.find('bluerov2_description', 'tilt.yaml')])           


    return sl.launch_description()
