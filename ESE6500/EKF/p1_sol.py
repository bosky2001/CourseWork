import numpy as np
import matplotlib.pyplot as plt

# step 1
np.random.seed(42)
a = -1
x = 1 + np.sqrt(2)*np.random.randn()
xs = [x]
ys = [np.sqrt(1+x**2) + np.sqrt(1/2.)*np.random.randn()]
T = 100
for i in range(T):
    x = a*x + np.random.randn()
    xs.append(x)
    ys.append(np.sqrt(1+x**2) + np.sqrt(1/2.)*np.random.randn())

# step 2
R = np.diag([1,0.1])
Q = 0.5
zs = [[1,-10]]; Ss = [np.diag([2, 2])]

for i in range(1,T):
    z, S = zs[-1], Ss[-1]

    zp = np.array([z[0]*z[1], z[1]])
    A = np.array([[z[1], z[0]], [0,1]])
    Sp = A @ S @ A.T + R

    yh = np.sqrt(1+zp[0]**2)
    C = np.array([zp[0]/yh, 0])
    K = Sp @ C.T/(C @ Sp @ C.T + Q)
    zpp = zp + K*(ys[i] - yh)
    Spp = (np.eye(2) - np.outer(K,C)) @ Sp

    zs.append(zpp)
    Ss.append(Spp)

zs, Ss = np.array(zs), np.array(Ss)

plt.figure(1); plt.clf();
plt.plot(xs, label='x')
plt.plot(a*np.ones((T)), label='a')
plt.plot(ys, label='y')
plt.plot(zs[:,0], label='xh')
plt.plot(zs[:,1], label='ah')
plt.legend()