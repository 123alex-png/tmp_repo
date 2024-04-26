uint16 task_id  # id of the task
string task_name  # name of the task
string task_type   # type of the task  bridge task  slope task road task 
Way_Point EnterPoint
Way_Point LeavePoint
geometry_msgs/Point32 position  # position of the task  x,y,z
string req_uav_type   # requirment uav  type   HighwayPatrol Agriculture Logistics
string req_payload_type  # Camera
uint16 execute_time
uint16 Ts        # The earliest execution time of the time window
uint16 Te        # The latest execution time of the time window
uint16 Ts_l     # Ts_l= Te - execute_time
uint16 Te_e     # Te_e = Ts+execute_time
float32 S_t       # real execute starttime
float32 E_t          # real execute endtime
uint16 R         # income of the task
uint16 done          # be allocated or not   0表示未分配  1表示已被分配  2 表示被分配但无满足无人机 3表示正在被执行  4表示已经被执行
int16[] allocated   # The uav accepting this task
geometry_msgs/Point32[] task_path
float32 path_length
