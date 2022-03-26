#coding=gbk
import os                          #导入os系统命令格式
format='json'                       #自定义要修改的文件格式
filenames=os.listdir(os.getcwd())  #进入当前目录，并把当前目录下的文件定义为filenames
for filename in filenames:         #对每一个文件进行遍历操作
    i=filename.split('.')          #用‘.’作为分割符，对文件名字符隔开
    if filename!='rename2.py' :
    #由于在操作中要把.py文件放到要修改的文件目录下，所以要过滤.py文件不要做该格式操作，同     时，如果该文件后缀是.zip也不做修改
        t=filename.split('_')       #再一次对文件进行分割，关键字符为‘_’
        os.rename(filename,t[0]+t[1])  #重命名文件