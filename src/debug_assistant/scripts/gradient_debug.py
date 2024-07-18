import numpy as np
import matplotlib.pyplot as plt

# 读取文件内容
data = np.loadtxt('/home/fast/Implicit_SDF_planner/src/debug_assistant/scripts/gradient_debug.txt')
print("data shape{}".format(data.shape))
# 提取x和f(x)的值
x = data[1:, 0]
f_x = data[1:, 1]
g_x = data[1:, 2]
ng_x = data[1:, 3]

x0 = data[0,0]
y0 = data[0,1]
g0 = data[0,2]
ng0 = data[0,3]

# 绘制图形
plt.plot(x, f_x, 'b',label='f(x)')
plt.plot(x, g_x, 'g',label='g(x)')
plt.plot(x, ng_x, 'r',label='ng(x)')
plt.plot(x0, y0,'ro')
plt.xlabel('x')
plt.ylabel('f(x)')
plt.title('f(x) vs. x')
plt.show()