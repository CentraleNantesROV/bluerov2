from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()
    
    sl.declare_arg('gui', default_value=True)

    with sl.group(if_arg='gui'):
        sl.gz_launch(f"-r {sl.find('bluerov2_ignition', 'demo_world.sdf')}")
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch(f"-s -r {sl.find('bluerov2_ignition', 'demo_world.sdf')}")
        
    sl.create_gz_clock_bridge()
        
    return sl.launch_description()
