#coding=gbk
import os                          #����osϵͳ�����ʽ
format='json'                       #�Զ���Ҫ�޸ĵ��ļ���ʽ
filenames=os.listdir(os.getcwd())  #���뵱ǰĿ¼�����ѵ�ǰĿ¼�µ��ļ�����Ϊfilenames
for filename in filenames:         #��ÿһ���ļ����б�������
    i=filename.split('.')          #�á�.����Ϊ�ָ�������ļ����ַ�����
    if filename!='rename2.py' :
    #�����ڲ�����Ҫ��.py�ļ��ŵ�Ҫ�޸ĵ��ļ�Ŀ¼�£�����Ҫ����.py�ļ���Ҫ���ø�ʽ������ͬ     ʱ��������ļ���׺��.zipҲ�����޸�
        t=filename.split('_')       #��һ�ζ��ļ����зָ�ؼ��ַ�Ϊ��_��
        os.rename(filename,t[0]+t[1])  #�������ļ�