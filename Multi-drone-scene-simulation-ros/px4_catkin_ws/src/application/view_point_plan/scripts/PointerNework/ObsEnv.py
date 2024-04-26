class ObsEnv():
    """
    该类用于描述环境障碍物
    """

    def __init__(self, name, id, x_range, y_range, z_range, Obstacles):
        self.name = name
        self.id = id
        self.x_range = x_range
        self.y_range = y_range
        self.z_range = z_range
        self.Obstacles = Obstacles  # min
        self.aabb_Obstacles = []
