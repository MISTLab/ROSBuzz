import re

pathfname = "/home/nvidia/output_path.txt"

outputfilename = "/home/nvidia/gbplanner_ws/src/control/rosbuzz/buzz_scripts/include/utils/nav_table.bzz"

ppoints_list_x = []
ppoints_list_y = []
with open(pathfname) as f:
    content = f.read()
    ppoints_list_x = re.findall('(?<=x: )-?\d*?\.\d+', content)
    ppoints_list_y = re.findall('(?<=y: )-?\d*?\.\d+', content)
f.close()


f = open(outputfilename, "w")
f.write("GLOBAL_TEST_PATH = {.1={.x=" + str(ppoints_list_x[0]) + ", .y=" + str(ppoints_list_y[0]) + " }")

for i in range(1,len(ppoints_list_x)):
    f.write(",."+str(i+1)+"={.x=" + str(ppoints_list_x[i]) + ", .y=" + str(ppoints_list_y[i]) + " }")
f.write(" }")

f.close()
