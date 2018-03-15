import matplotlib.pyplot as plt
import numpy as np
from scipy import optimize

data=[np.arange(0,20.0,1),np.array([-52.368, 32.221, 40.102, 48.088, 73.106, 50.807, 52.235, 76.933, 65.737, 34.772, 94.376, 123.366, 92.71, 72.25, 165.051, 91.501, 118.92, 100.936, 56.747, 159.034])]

def line(m,b):
   return np.arange(0, 20, 1)*m + b

guess = np.ones(2)

errfunc= lambda p,y: (y-line(p[0],p[1]))
parameter, sucess = optimize.leastsq(errfunc, guess, args=(data[1]))

print(parameter)
plt.plot(data[0],data[1],'o')
plt.plot(data[0],line(parameter[0],parameter[1]))
plt.show()