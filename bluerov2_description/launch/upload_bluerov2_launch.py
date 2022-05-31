from simple_launch import SimpleLauncher, IgnitionBridge

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('ground_truth',default_value=True)
    sl.declare_arg('sliders',default_value=True)
    
    # initial pose
    sl.declare_arg('x', 1.)
    sl.declare_arg('y', 0.)
    sl.declare_arg('z', 1.)
    sl.declare_arg('roll', 0.)
    sl.declare_arg('pitch', 0.)
    sl.declare_arg('yaw', 0.)
    
    namespace=sl.arg('namespace')
    
    # robot state publisher
    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': namespace, 'use_sim_time': sl.sim_time})
               
    with sl.group(ns=namespace):
                    
        # URDF spawner to ignition, defaults to relative robot_description topic
        sl.spawn_ign_model(namespace, spawn_args = sl.gazebo_axes_args())
            
        # spawn ign / ros bridge anyway        
        bridges = []
        ign_js_topic = sl.name_join(IgnitionBridge.model_prefix(namespace),'/joint_state')
        bridges.append(IgnitionBridge(ign_js_topic, 'joint_states', 'sensor_msgs/JointState', IgnitionBridge.ign2ros))
        
        bridges.append(IgnitionBridge(sl.name_join('/model/', namespace, '/pose'),
                                     'pose_gt', 'geometry_msgs/Pose', IgnitionBridge.ign2ros))
        
        # thrusters
        for thr in range(1, 7):
            thruster = f'thruster_{thr}'
            ign_thr_topic = sl.name_join('/model/', namespace, f'/joint/{thruster}/cmd_thrust')
            bridges.append(IgnitionBridge(ign_thr_topic, f'cmd_thruster_{thr}', 'std_msgs/Float64', IgnitionBridge.ros2ign))        
        
        sl.create_ign_bridge(bridges, 'ign_bridge')
                        
        # ground truth to tf if requested
        with sl.group(if_arg='ground_truth'):
            sl.node('pose_to_tf',parameters={'child_frame': sl.name_join(namespace, '/base_link')})
            
        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', arguments=[sl.find('bluerov2_description', 'thrusters.yaml')])
    
    return sl.launch_description()
