from data import a 

import math
##

# print (a)
arr=[]
temp = 0
k = 0 #7613
width = 377
height = 392
for i in range(height):
    onerow = []
    for j in range(width):
        onerow.append(a[k])
        k = k + 1
    arr.append(onerow)
##
arr2=[]

for i in range(width):
    row = []
    for j in range(height):
        row.append(arr[height-1-j][width-1-i])
    arr2.append(row)

for i in range(width):
    for j in range(height):
        if (arr2[i][j] == -1):
            arr2[i][j] = "x"
        if (arr2[i][j] == 0):
            arr2[i][j] = "-"
        if (arr2[i][j] == 100):
            arr2[i][j] = ":"
    #     print(arr2[i][j],end="")
    # print("|")
#print((arr2))
##
init_map = [-9.298303, -9.747517]
init_pose = [-8.0, 8.0]
max_pixel = 10
step = 2

def oxy2pixel(x,y):
    pixel = []
    pixel.append(width-1-int(((x-init_map[0])/0.05)))
    pixel.append(height-1-int(((y-init_map[1])/0.05)))
    return pixel

def pixel2oxy(x,y):
    oxy = []
    oxy.append(init_map[0]+(width-1-x)*0.05)
    oxy.append(init_map[1]+(height-1-y)*0.05)
    return oxy
#print(oxy2pixel(-8,8))
def dis_pixel(x1,y1,x2,y2):
    dis = 0.05*(math.sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2)))
    return dis

def ang_pixel(x1,y1,x2,y2):
    ang =0 
    if (x1 == x2):
        if (y1>y2):
            ang = 180
        if (y1<y2):
            ang = 0
    else:
        ang = int((math.atan(abs(y2-y1)/abs(x1-x2)))*180/math.pi)
        if (y1>=y2):
            ang = ang+90
        else: 
            ang = 90- ang
    return ang

arr3=[]
def find_obj(x,y):
    pose = oxy2pixel(x,y)
    print(pose)
    for i in range (0, max_pixel, step):
        row1=[]
        for j in range(max_pixel):
            if (pose[0]-i<0 or pose[1]+j>height):
                break
            if (arr2[pose[0]-i][pose[1]+j]==':'):
                dis1 = dis_pixel(pose[0],pose[1],pose[0]-i,pose[1]+j)
                ang1 = ang_pixel(pose[0],pose[1],pose[0]-i,pose[1]+j)
                pose_oxy1 = pixel2oxy(pose[0]-i,pose[1]+j)
                row1.append(pose[0]-i)
                row1.append(pose[1]+j)

                row1.append(dis1)
                row1.append(ang1)
                row1.append(pose_oxy1[0])
                row1.append(pose_oxy1[1])
                arr2[pose[0]-i][pose[1]+j] = "O"
                break
        row2=[]
        for k in range(max_pixel):
            if (pose[0]-i<0 or pose[1]-k<0):
                break
            if (arr2[pose[0]-i][pose[1]-k]==':'):
                dis2 = dis_pixel(pose[0],pose[1],pose[0]-i,pose[1]-k)
                ang2 = ang_pixel(pose[0],pose[1],pose[0]-i,pose[1]-k)
                pose_oxy2 = pixel2oxy(pose[0]-i,pose[1]-k)
                row2.append(pose[0]-i)
                row2.append(pose[1]-k)
                row2.append(dis2)
                row2.append(ang2)
                row2.append(pose_oxy2[0])
                row2.append(pose_oxy2[1])
                arr2[pose[0]-i][pose[1]-k] = "O"
                break
        if (row1 != []):
            arr3.append(row1)
        if (row2 != []):
            arr3.append(row2)
    print(f'sum keys : {len(arr3)}')
    print(arr3)
    for i in range(width):
        for j in range(height):
            print(arr2[i][j],end="")
        print("|")
    for i in arr3: 
        print (i)
    # print(arr3)
#print(arr2)
find_obj(-6,-7)




