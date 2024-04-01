import time
import torch
import argparse

device = torch.device('cuda:0') if torch.cuda.is_available() else torch.device('cpu')
# device = torch.device('cpu')
time_now = time.strftime('%y%m_%d%H%M')
def argparser():
    parser = argparse.ArgumentParser()
    
    parser.add_argument("__name", default="view_plan", help="node name")
    parser.add_argument("__log", default="", help="ros node log")
    parser.add_argument("--device", default=device, help="CPU or GPU")
    parser.add_argument("--mode",type=str,default="train",help="运行模式[train, train_emv, test]")
    parser.add_argument("--target_point_num",type=int,default=20,help="number of view points" )
    parser.add_argument("--max_episode", type=int, default=20000, help="number of episode")
    parser.add_argument("--num_samples", type=int, default=3, help="number of episode")
    parser.add_argument("--use_rrt",type=bool, default=False,help=" use rrt to path plan")
    # 训练参数
    parser.add_argument("--lr",type=float, default=0.001, help="学习率")
    parser.add_argument("--gamma",type=float, default=0.95, help="折扣因子")
    parser.add_argument("--batch_size", type=int, default=512, help="同一时刻优化的episode数")  # default=1024
    parser.add_argument("--num_units", type=int, default=128, help="mlp的单元(神经元)数量")
    parser.add_argument("--tao", type=float, default=0.01, help="number of adversaries")

    # path
    parser.add_argument('--subTask_filename',type=str,default='/home/zrh/px4_catkin_ws1019/src/application/view_point_plan/scripts/data/subTask.txt')
    parser.add_argument('--obstacles_filename',type=str,default='/home/zrh/px4_catkin_ws1019/src/application/view_point_plan/scripts/data/Obstacles.txt')
    parser.add_argument('--islogger', action='store_false', help='flag csv logger default true')
    parser.add_argument('--issaver', action='store_false', help='flag model saver default true')
    parser.add_argument('--log_step', type=int, default=10, help='logger timing')
    parser.add_argument('--log_dir',type=str, default='../Csv/', help='csv logger dir')
    parser.add_argument('--model_dir', type=str, default='../Pt/', help='model save dir')
    parser.add_argument('--pkl_dir', type=str, default='./Pkl/', help='pkl save dir')
    parser.add_argument('--path', type=str, default='Pkl/test20.pkl', help='pkl file name')

    return parser.parse_args()


if __name__ == '__main__':

    args = argparser()
