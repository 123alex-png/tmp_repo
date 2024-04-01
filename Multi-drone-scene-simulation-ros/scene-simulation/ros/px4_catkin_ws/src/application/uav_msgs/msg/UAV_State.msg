uint16 uav_id   # id of the UAV 
string uav_name  # name of the UAV
string uav_type   # HighwayPatrol Agriculture Logistics
string payload_type  # Camera
geometry_msgs/Point32 uav_start_pos
geometry_msgs/Point32 ground_station
geometry_msgs/Point32 position  # pos of the UAV
geometry_msgs/Point32 velocity  # v of the UAV
float32 velocity_size   
int8 work_temperature_min  # min of work temperature
int8 work_temperature_max  # max ..
float32 flight_time  # flight time
float32 power       # power
float32 max_comm_length  # max communication length
float32 max_WR    # max wind speed
float32 wl        # water level

uint16[] accepted  # accepted tasks' list
uint8 bid_length  # current bid length
uint8[] bid_task_list  # bid_task_list 
float32[] bid_task_price  # bid_task_price
uint16 work_state   # 0:take off 1:fly  2:land  3:idle  4:invalid

