# -*- coding: utf-8 -*-
# Dos cartas de esferas (tutorial Yade: inAlignedBox + regularOrtho)

from __future__ import print_function
from yade import pack, utils, qt
from yade import *
from math import pi, sin

# --- geometria (m) ---
CARD_LENGTH = 63.0e-3
CARD_WIDTH = 88.0e-3
CARD_THICKNESS = 2.0e-3       # espesor t
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t, una capa en z
DIAMOND_RX = 0.12 * CARD_LENGTH
DIAMOND_RY = 0.12 * CARD_WIDTH

# --- fisica ---
m_card = FrictMat(density=800, young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)

# predicado: caja (0,0,0) -> (L,W,2r)  como en el tutorial
pred = pack.inAlignedBox(
	(0, 0, 0),
	(CARD_LENGTH, CARD_WIDTH, 2.0 * SPHERE_RADIUS),
)


def sphere_color(x, y, z):
	if abs(z - SPHERE_RADIUS) > 0.6 * SPHERE_RADIUS:
		return (1, 1, 1)
	dx = abs(x - CARD_LENGTH / 2.0) / DIAMOND_RX
	dy = abs(y - CARD_WIDTH / 2.0) / DIAMOND_RY
	if dx + dy < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(tilt, x0, label):
	"""tilt: rotacion alrededor de x. x0: desplazamiento en x del centro."""
	rot = Quaternion((1, 0, 0), tilt)
	pivot = Vector3(CARD_LENGTH / 2.0, 0, SPHERE_RADIUS)
	n = 0
	for b in pack.regularOrtho(pred, radius=SPHERE_RADIUS, gap=0, material=m_card):
		p = b.state.pos
		rel = Vector3(p[0], p[1], p[2]) - pivot
		wp = rot * rel + pivot + Vector3(x0, SPHERE_RADIUS, 0)
		b.state.pos = wp
		b.shape.color = sphere_color(p[0], p[1], p[2])
		O.bodies.append(b)
		n += 1
	print('Carta', label, ':', n, 'esferas')


# suelo
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1, material=m_floor, color=(0.4, 0.4, 0.4))
)

# dos cartas en /\  (30 deg desde la vertical)
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
