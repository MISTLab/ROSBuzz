import re

filename = "/home/vivek/Vivek/Projects/Hierarchical_Swarm/KheperaIV/data_processing/Hirarchial_exp_log/2_Robot/3/RG1/log/nav_tube.txt"
pathfname = "/home/vivek/Vivek/Projects/Hierarchical_Swarm/KheperaIV/data_processing/Hirarchial_exp_log/2_Robot/3/RG1/log/path.txt"

outputfilename ="/home/vivek/Vivek/Projects/Hierarchical_Swarm/KheperaIV/data_processing/nav_table.bzz"

points_list = []
with open(filename) as f:
    #content = f.readlines()
# you may also want to remove whitespace characters like `\n` at the end of each line
#content = [x.strip() for x in content] 
    content = f.read()
    points_list_x = re.findall('(?<=x: )-?\d*?\.\d+', content)
    points_list_y = re.findall('(?<=y: )-?\d*?\.\d+', content)
f.close()

print(len(points_list_x))
print(len(points_list_y))

ppoints_list_x = []
ppoints_list_y = []
with open(pathfname) as f:
    content = f.read()
    ppoints_list_x = re.findall('(?<=x: )-?\d*?\.\d+', content)
    ppoints_list_y = re.findall('(?<=y: )-?\d*?\.\d+', content)
f.close()


counter = 0
left = []
right = []
while (counter < len(points_list_x)):
    if ((counter) % 2 == 0):
        left.append([points_list_x[counter],0])
    if ((counter - 1) % 2 == 0):
        right.append([points_list_x[counter],0])
    counter = counter + 1

counter = 0
while (counter < len(points_list_y)):
    if ((counter) % 2 == 0):
        left[int(counter/2)][1] = points_list_y[counter]
    if ((counter - 1) % 2 == 0):
        right[int((counter-1)/2)][1] = points_list_y[counter]
    counter = counter + 1

f = open(outputfilename, "w")
f.write("Epsilon_tube = {.1"+"={")
f.write(".1"+"={.x=" + str(left[0][0]) + ", .y=" + str(left[0][1]) + " },")
f.write(".2"+"={.x=" + str(right[0][0]) + ", .y=" + str(right[0][1]) + " }")
f.write(" }")
for i in range(1,len(left)):
    f.write(",."+str(i+1)+"={")
    f.write(".1"+"={.x=" + str(left[i][0]) + ", .y=" + str(left[i][1]) + " },")
    f.write(".2"+"={.x=" + str(right[i][0]) + ", .y=" + str(right[i][1]) + " }")
    f.write(" }")
    print("Diff X:"+str( float(left[i][0]) - float(right[i][0])) +" Y: "+str(float(left[i][1]) - float(right[i][1]) ))
f.write(" }")

f.write(" \n")
f.write(" \n")
f.write(" \n")

f.write("GLOBAL_TEST_PATH = {.1={.x=" + str(ppoints_list_x[0]) + ", .y=" + str(ppoints_list_y[0]) + " }")

for i in range(1,len(ppoints_list_x)):
    f.write(",."+str(i+1)+"={.x=" + str(ppoints_list_x[0]) + ", .y=" + str(ppoints_list_y[0]) + " }")
f.write(" }")

f.close()
