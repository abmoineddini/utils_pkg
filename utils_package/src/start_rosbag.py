import rospy
import time
import os
from datetime import datetime
import subprocess

dirname = datetime.now().strftime("%Y%m%d")

save_path = '/workdir/notebooks/dev_ws/src/collection_logs/rosbag_'+dirname
os.makedirs(save_path,exist_ok=True)
testname = save_path+"/test_"+str(datetime.now()).split(" ")[1]+".bag"
print(testname)
rospy.init_node("start_rosbag", anonymous = True)
subprocess.call(f'rosbag record /brainbox3_control/Dout0/conv_1 /brainbox3_control/Dout1/conv_2 /brainbox3_control/Dout2/conv_3 /brainbox3_control/Dout3/conv_4 /brainbox3_control/Dout4/conv_6 /brainbox3_input/sensor_input1/breakbeam2 /brainbox3_input/sensor_input2/breakbeam3 /brainbox3_input/sensor_input3/reset_system /brainbox3_input/sensor_input4/breakbeam4 /conveyor_control/conveyor_1 /brainbox1_input/sensor_input4/breakbeam1 /conveyor_feedback/breakbeam_4 /brainbox1_input/sensor_input2/breakbeam5 /denester/arm_availability /denester/control /denester/feedback /image_processing/camera_1_bbox1 /image_processing/camera_1_bbox2 /image_processing/cutting_board_clearance_status /image_processing/qa_positioning /qa_control/qa_availibity /qa_control/qa_processing /qa_control/qa_status /reset_system /rosout /size_truss/breakbeam_1 /size_truss/qa /subdiv_control/activate /subdiv_control/blade /subdiv_control/completion /subdiv_control/cutting_board /subdiv_control/deactivate /subdiv_control/pusher /subdiv_control/subdiv_truss_available /subdiv_feedback/activate /subdiv_feedback/blade /subdiv_feedback/cutting_board /subdiv_feedback/pusher /top_up_control/scale /top_up_control/turntable /top_up_control/unloading/turntable1 /top_up_control/unloading/turntable2 /top_up_feedback/scale /top_up_feedback/unloading/turntable1 /top_up_feedback/unloading/turntable2 /transition_control/post_subdiv /transition_control/pre_subdiv /tray_unloading/control_unloading /tray_unloading/truss_availability -O {testname}', shell=True)



 #