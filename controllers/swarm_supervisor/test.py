import numpy as np
import geatpy as ea
# 定义种群规模（个体数目）
x1 = [-3, 12.1] # 第一个决策变量范围
x2 = [4.1, 5.8] # 第二个决策变量范围
b1 = [1, 1] # 第一个决策变量边界，1表示包含范围的边界，0表示不包含
b2 = [1, 1] # 第二个决策变量边界，1表示包含范围的边界，0表示不包含
ranges=np.vstack([x1, x2]).T # 生成自变量的范围矩阵，使得第一行为所有决策变量的下界，第二行为上界
borders=np.vstack([b1, b2]).T # 生成自变量的边界矩阵
Nind = 4
Encoding = 'BG' # 表示采用“实整数编码”，即变量可以是连续的也可以是离散的
varTypes = np.array([0, 0])
codes = [0, 0] # 决策变量的编码方式，设置两个0表示两个决策变量均使用二进制编码
precisions =[1, 1] # 决策变量的编码精度，表示二进制编码串解码后能表示的决策变量的精度可达到小数点后6位
scales = [0, 0] # 0表示采用算术刻度，1表示采用对数刻度
FieldD = ea.crtfld(Encoding,varTypes,ranges,borders,precisions,codes,scales)
# 创建“译码矩阵”
# FieldD = np.array([[3, 2], # 各决策变量编码后所占二进制位数，此时染色体长度为3+2=5
#                    [0, 0], # 各决策变量的范围下界
#                    [7, 3], # 各决策变量的范围上界
#                    [0, 0], # 各决策变量采用什么编码方式(0为二进制编码，1为格雷编码)
#                    [0, 0], # 各决策变量是否采用对数刻度(0为采用算术刻度)
#                    [1, 1], # 各决策变量的范围是否包含下界(对bs2int实际无效，详见help(bs2int))
#                    [1, 1], # 各决策变量的范围是否包含上界(对bs2int实际无效)
#                    [0, 0]])# 表示两个决策变量都是连续型变量（0为连续1为离散）
# 调用crtri函数创建实数值种群
Chrom = ea.crtpc(Encoding, Nind, FieldD)
x = ea.bs2real(Chrom,FieldD)
print(list(x[1,:]))

