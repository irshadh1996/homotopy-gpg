import numpy as np
CMD_VEL_MESSAGE_TYPE = "geometry_msgs/Twist"
ODOM_MESSAGE_TYPE = "nav_msgs/Odometry"
ROBOT_CONFIGS = [
        {
        'id': 1,
        'websocket_url': "ws://localhost:9090", # Example: Robot 2 on a different IP
        'odom_topic': "/robot_1/odom",
        'cmd_vel_topic': "/robot_1/cmd_vel"
    },
    {
        'id': 2,
        'websocket_url': "ws://localhost:9090", # Example: Robot 2 on a different IP
        'odom_topic': "/robot_2/odom",
        'cmd_vel_topic': "/robot_2/cmd_vel"
    },
    {
        'id': 3,
        'websocket_url': "ws://localhost:9090", # Example: Robot 2 on a different IP
        'odom_topic': "/robot_3/odom",
        'cmd_vel_topic': "/robot_3/cmd_vel"
     },
     # {
    #     'id': 4,
    #     'websocket_url': "ws://192.168.1.130:9090", # Example: Robot 2 on a different IP
    #     'odom_topic': "/robot_4/odom",
    #     'cmd_vel_topic': "/robot_4/cmd_vel"
    # },
    # {
    #     'id': 5,
    #     'websocket_url': "ws://192.168.1.130:9090", # Example: Robot 2 on a different IP
    #     'odom_topic': "/robot_5/odom",
    #     'cmd_vel_topic': "/robot_5/cmd_vel"
    # }    
    ]
N_players = len(ROBOT_CONFIGS) # Dynamically set N_players based on config
def get_robot_config_by_id(robot_id):
    """Helper to get config from ROBOT_CONFIGS list by ID."""
    for config in ROBOT_CONFIGS:
        if config['id'] == robot_id:
            return config
    return None
