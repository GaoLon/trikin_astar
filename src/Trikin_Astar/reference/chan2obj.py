import re
import random
def off2obj(offpath, objpath):
    '''
    将obj文件转换为off文件
    :param objpath: .obj文件的路径
    :param offpath: .off文件的路径的保存地址
    :return: 无
    '''
    line = ""

    vset = []
    fset = []
    vnum = 0
    fnum = 0
    with open(offpath,'r') as f:
        lines = f.readlines()
    p = re.compile(r'/+')
    space = re.compile(r' +')
    cnt=0

    for line in lines:
        if cnt==0:
            cnt+=1
            continue
        if cnt==1:
            cnt+=1
            parameters = space.split(line.strip())
            vnum = eval(parameters[0])
            fnum = eval(parameters[1])
            continue
        if cnt==2:
            cnt+=1
            continue
        if cnt<vnum+3:
            cnt+=1
            parameters = space.split(line.strip())
            Point = []
            Point.append(eval( parameters[0]) )
            Point.append(eval( parameters[1]) )
            Point.append(eval( parameters[2]) )
            vset.append(Point)
            continue
        parameters = space.split(line.strip())
        cnt+=1
        if cnt==vnum+fnum+3:
            break
        if eval(parameters[0]) == 3:
            vIndexSets = []
            for i in range(3):
                index = eval(parameters[i+1])
                index += 1
                vIndexSets.append(index)
            fset.append(vIndexSets)

    with open(objpath, 'w') as out:
        out = open(objpath, 'w')
        for j in range(len(vset)):
            out.write("v " + str(vset[j][0]) + " " + str(vset[j][1]) + " " + str(vset[j][2]) + "\n")
        out.write("\n")
        for i in range(len(fset)):
            s = "f"
            # s = s + " " + str(fset[i][0])+ " " + str(fset[i][2])+ " " + str(fset[i][1])
            for j in range( len( fset[i] ) ):
                s = s+ " "+ str(fset[i][j])
            s += "\n"
            out.write(s)

    print("{} 转换成 {} 成功！".format( p.split(offpath)[-1], p.split(objpath)[-1] ))


if __name__ == '__main__':
    off2obj('out.off','out.obj')