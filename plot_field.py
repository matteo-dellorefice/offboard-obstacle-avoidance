import numpy as np
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import math
import matplotlib.animation as animation

walls = np.array([
    [ [-8., 5.], [2., 5.] ],
    [ [5., 10.], [5., -10.] ],
    [ [12., 10.], [12., -10.] ],
    [ [5., -10.], [21.5, -10.] ]
])

target = np.array([-10, 10])

def segment_distance_field(a, b, p):
    l2 = np.linalg.norm(a - b) ** 2
    
    if l2 == 0: return np.linalg.norm(p - a)

    t = max(0., min(1., np.dot(p - a, b - a) / l2))
    proj = a + t * (b - a)

    return np.linalg.norm(p - proj)

def gauss(mu, ampl, spread, x):
    return ampl * math.exp((-1 * (x - mu) ** 2) / spread)

    # double coeff = max_step * (ftype - exp((-1. * pow(norm, 2.)) / spread));

def inv_power(ampl, x):
    return ampl * (1 / abs(x)) ** 2

def gauss2d(mu, ampl, spread, x):
    return ampl * math.exp((-1 * ((x[0] - mu[0])** 2 + (x[1] - mu[1]) ** 2)) / spread)

def target_field(t, p):
    ampl = 0.2
    spread = 300.
    return ampl - gauss(0, ampl, spread, np.linalg.norm(t - p))

# def gauss(x, mu, sigma):
#     coeff = 1 / (sigma * math.sqrt(2 * math.pi))
#     expon = -1 * (x - mu) ** 2 / ( 2 * sigma ** 2)
#     return coeff * math.exp(expon)

def field_total(p):
    components = [inv_power(1, segment_distance_field(w[0], w[1], p)) for w in walls]
    components.append(target_field(target, p))
    return max(components)

x = np.linspace(-10, 24, 512)
y = np.linspace(-12, 12, 512)
X, Y = np.meshgrid(x, y)
data = np.dstack([X, Y]).reshape(512 * 512, 2)
Z = np.apply_along_axis(field_total, 1, data)
Z = Z.reshape(512, 512)

fig = plt.figure()
ax = fig.add_subplot(projection='3d')
ax.plot_surface(X, Y, Z, cmap=cm.viridis,linewidth=0,antialiased=False)
# ax.set_aspect('equal')
ax.set_zlim(-0.5, 1)
#labelling
ax.set_xlabel('X', labelpad=20)
ax.set_ylabel('Y', labelpad=20)
ax.set_zlabel('Z', labelpad=20)
# ax.set_title("")

def init():
    ax.view_init(30, 50, 0)
    return fig

def animate(i):
    ax.view_init(30, i + 50, 0)
    return fig

# ani = animation.FuncAnimation(fig, animate, init_func=init, repeat=True, frames=360, interval=20)

# writer = animation.PillowWriter(fps=15,
#                                 metadata=dict(artist='Me'),
#                                 bitrate=1800)
# ani.save('field.gif', writer=writer)

plt.show()