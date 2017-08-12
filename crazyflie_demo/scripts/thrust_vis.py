import numpy as np
#import matplotlib
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from math import pi
import time

class ThrustVis:
	def __init__(self):

		# how big to draw it on screen
		SCALE = 100

		# actual geometry in meters
		radius = 0.23
		prop_rad = 0.0254 * (6 / 2.0)
		thetas = np.array([pi/6 + i*(pi/3) for i in range(6)])
		xs = radius * np.cos(pi/2 - thetas)
		ys = radius * np.sin(pi/2 - thetas)

		def make_circle(x, y):
			return plt.Circle([x, y], prop_rad, 
				facecolor=self._color(0),
				edgecolor=[0, 0, 0])

		self.circles = [make_circle(x, y) for x, y in zip(xs, ys)]

		box_lim = radius + 1.5 * prop_rad
		lim = [-box_lim, box_lim]

		self.fig = plt.figure()
		ax1 = self.fig.add_subplot(111,
			aspect='equal',
			xlim=lim,
			ylim=lim,
		)

		for c in self.circles:
			ax1.add_patch(c)

		self.fig.show()


	def set_thrusts(self, thrusts):
		for c, t in zip(self.circles, thrusts):
			c.set_facecolor(self._color(t))
		self.fig.canvas.draw()


	def _color(self, thrust):
		GY = 0.3
		def lerp(a, b):
			return (1.0 - thrust) * a + thrust * b
		return [lerp(GY, 1), lerp(GY, 0), lerp(GY, 0)]


if __name__ == "__main__":
	v = ThrustVis()
	time.sleep(1.0)
	v.set_thrusts([(1.0 / 5.0) * i for i in range(6)])
	time.sleep(1.0)
