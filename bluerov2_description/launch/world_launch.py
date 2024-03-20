from simple_launch import SimpleLauncher


def generate_launch_description():
    
    sl = SimpleLauncher()
    sl.declare_arg('gui', default_value=True)

    with sl.group(if_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_description', 'demo_world.sdf'), "-r")
        
    with sl.group(unless_arg='gui'):
        sl.gz_launch(sl.find('bluerov2_description', 'demo_world.sdf'), "-r -s")
        
    sl.create_gz_clock_bridge()
        
    return sl.launch_description()
