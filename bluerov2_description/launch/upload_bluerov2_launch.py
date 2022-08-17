from simple_launch import SimpleLauncher, GazeboBridge

def generate_launch_description():
    
    sl = SimpleLauncher(use_sim_time = True)
    
    sl.declare_arg('namespace', default_value='bluerov2')
    sl.declare_arg('ground_truth',default_value=True)
    sl.declare_arg('sliders',default_value=True)
    sl.declare_arg('camera', False)
    
    # initial pose
    sl.declare_gazebo_axes(x=1., y=0., z=1., roll=0.,pitch=0., yaw=0.)
    
    namespace=sl.arg('namespace')
    
    # robot state publisher
    sl.include('bluerov2_description', 'state_publisher_launch.py',
               launch_arguments={'namespace': namespace, 'use_sim_time': sl.sim_time, 'camera': sl.arg('camera')})
               
    with sl.group(ns=namespace):
                    
        # URDF spawner to Gazebo, defaults to relative robot_description topic
        sl.spawn_gz_model(namespace, spawn_args = sl.gazebo_axes_args())
            
        # spawn gz / ros bridge anyway        
        bridges = []
        gz_js_topic = sl.name_join(GazeboBridge.model_prefix(namespace),'/joint_state')
        bridges.append(GazeboBridge(gz_js_topic, 'joint_states', 'sensor_msgs/JointState', GazeboBridge.gz2ros))
        
        # pose ground truth
        bridges.append(GazeboBridge(sl.name_join('/model/', namespace, '/pose'),
                                     'pose_gt', 'geometry_msgs/Pose', GazeboBridge.gz2ros))
        
        # odometry
        bridges.append(GazeboBridge(sl.name_join('/model/', namespace, '/odometry'),
                                     'odom', 'nav_msgs/Odometry', GazeboBridge.gz2ros))
        
        # camera
        with sl.group(if_arg='camera'):
            bridges.append(GazeboBridge(sl.name_join(namespace, '/image'), 'image', 'sensor_msgs/Image', GazeboBridge.gz2ros))
        
        # tilt control
        bridges.append(GazeboBridge(sl.name_join('/model/', namespace, '/joint/tilt/0/cmd_pos'),
                                     'cmd_tilt', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        # thrusters
        for thr in range(1, 7):
            thruster = f'thruster{thr}'
            gz_thr_topic = sl.name_join('/model/', namespace, f'/joint/{thruster}/cmd_thrust')
            bridges.append(GazeboBridge(gz_thr_topic, f'cmd_{thruster}', 'std_msgs/Float64', GazeboBridge.ros2gz))
        
        sl.create_gz_bridge(bridges)
                        
        # ground truth to tf if requested
        with sl.group(if_arg='ground_truth'):
            sl.node('pose_to_tf',parameters={'child_frame': sl.name_join(namespace, '/base_link')})
        # otherwise publish ground truth as another link to get error
        with sl.group(unless_arg='ground_truth'):
            sl.node('pose_to_tf',parameters={'child_frame': sl.name_join(namespace, '/base_link_gt')})
                        
        with sl.group(if_arg='sliders'):
            sl.node('slider_publisher', arguments=[sl.find('bluerov2_description', 'manual.yaml')])
    
    return sl.launch_description()
