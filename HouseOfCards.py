# -*- coding: utf-8 -*-
# Dos cartas = rejilla de esferas (sin pack: la caja en z=2r no sirve con regularOrtho)

from __future__ import print_function
from yade import utils, qt
from yade import *
from math import pi, sin

# --- geometria (m) ---
CARD_LENGTH = 63.0e-3
CARD_WIDTH = 88.0e-3
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t, una sola capa en z = r
DIAMOND_RX = 0.12 * CARD_LENGTH
DIAMOND_RY = 0.12 * CARD_WIDTH

# --- fisica ---
m_card = FrictMat(density=800, young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)


def sphere_color(x, y):
	dx = abs(x - CARD_LENGTH / 2.0) / DIAMOND_RX
	dy = abs(y - CARD_WIDTH / 2.0) / DIAMOND_RY
	if dx + dy < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(tilt, x0, label):
	rot = Quaternion((1, 0, 0), tilt)
	pivot = Vector3(CARD_LENGTH / 2.0, 0, SPHERE_RADIUS)
	step = 2.0 * SPHERE_RADIUS
	n = 0
	x = SPHERE_RADIUS
	while x <= CARD_LENGTH - SPHERE_RADIUS + 1e-9:
		y = SPHERE_RADIUS
		while y <= CARD_WIDTH - SPHERE_RADIUS + 1e-9:
			rel = Vector3(x, y, SPHERE_RADIUS) - pivot
			wp = rot * rel + pivot + Vector3(x0, SPHERE_RADIUS, 0)
			O.bodies.append(
				sphere(
					wp,
					SPHERE_RADIUS,
					material=m_card,
					color=sphere_color(x, y),
				)
			)
			n += 1
			y += step
		x += step
	print('Carta', label, ':', n, 'esferas')


# suelo
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1, material=m_floor, color=(0.4, 0.4, 0.4))
)

sep = CARD_WIDTH * sin(pi / 6.0) / 2.0
add_card(pi / 6.0, -sep, 'A')
add_card(-pi / 6.0, sep, 'B')

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
