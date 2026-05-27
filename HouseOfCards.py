# -*- coding: utf-8 -*-
# Dos cartas rigidas (clump de esferas) en /\, suelo + gravedad

from yade import utils, qt
from yade import *
from math import pi, sin, sqrt

# --- geometria (m) ---
CARD_LENGTH = 63.0e-3
CARD_WIDTH = 88.0e-3
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS
DIAMOND_RX = 0.10 * CARD_LENGTH
DIAMOND_RY = 0.10 * CARD_WIDTH

# --- fisica ---
m_card = FrictMat(density=800, young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)


def sphere_color(x, y):
	# rombo (45 deg en la cara), no cuadrado alineado a ejes
	cx = x - CARD_LENGTH / 2.0
	cy = y - CARD_WIDTH / 2.0
	u = (cx + cy) / sqrt(2.0)
	v = (-cx + cy) / sqrt(2.0)
	if abs(u) / DIAMOND_RX + abs(v) / DIAMOND_RY < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(tilt, x0):
	rot = Quaternion((1, 0, 0), tilt)
	pivot = Vector3(CARD_LENGTH / 2.0, 0, SPHERE_RADIUS)
	step = 2.0 * SPHERE_RADIUS
	members = []
	x = SPHERE_RADIUS
	while x <= CARD_LENGTH - SPHERE_RADIUS + 1e-9:
		y = SPHERE_RADIUS
		while y <= CARD_WIDTH - SPHERE_RADIUS + 1e-9:
			rel = Vector3(x, y, SPHERE_RADIUS) - pivot
			wp = rot * rel + pivot + Vector3(x0, SPHERE_RADIUS, 0)
			members.append(
				sphere(
					wp,
					SPHERE_RADIUS,
					material=m_card,
					color=sphere_color(x, y),
				)
			)
			y += step
		x += step
	O.bodies.appendClumped(members)


O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1, material=m_floor, color=(0.4, 0.4, 0.4))
)

sep = CARD_WIDTH * sin(pi / 6.0) / 2.0
add_card(pi / 6.0, -sep)
add_card(-pi / 6.0, sep)

O.engines = [
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Wall_Aabb()]),
	InteractionLoop(
		[Ig2_Sphere_Sphere_ScGeom(), Ig2_Wall_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_FrictPhys()],
		[Law2_ScGeom_FrictPhys_CundallStrack()],
	),
	NewtonIntegrator(damping=0.35, gravity=(0, -9.81, 0)),
]
O.dt = utils.PWaveTimeStep() * 0.4

V = qt.View()
V.viewDir = (1, 1, 1)
V.center()
