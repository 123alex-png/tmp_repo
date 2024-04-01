


class SubTask():
    def __init__(self,id,task_type,parent_task_id,target_name,target_id,EnterPoint,LeavePoint,target_start,target_end,point_list):
        self.id = id
        self.task_type =task_type
        self.target_name = target_name
        self.parent_task_id = parent_task_id
        self.target_id = target_id      # 用来找桥梁id
        self.EnterPoint = EnterPoint
        self.LeavePoint = LeavePoint
        self.target_start = target_start
        self.target_end = target_end
        self.point_list = point_list
        self.target_point_list = []
        self.path_list = []
        self.path_length = 0



