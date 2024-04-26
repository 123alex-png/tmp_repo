uint16 task_id  # id of the task
string task_name  # name of the task
string task_type   # type of the task  
Way_Point EnterPoint
Way_Point LeavePoint
geometry_msgs/Point32 position  # position of the task  x,y,z
string req_uav_type   # requirment uav  type   
uint16 Ts        # The earliest execution time of the time window
uint16 Te        # The latest execution time of the time window
bool done          # be allocated or not
int8[] allocated   # The uav accepting this task
geometry_msgs/Point32[] task_path
float32 path_length
