# -*- coding: utf-8 -*-
# Dos cartas rigidas (clump) - posicion y orientacion explicitas

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


def sphere_color(x_face, y_face):
	cx = x_face - CARD_LENGTH / 2.0
	cy = y_face - CARD_WIDTH / 2.0
	u = (cx + cy) / sqrt(2.0)
	v = (-cx + cy) / sqrt(2.0)
	if abs(u) / DIAMOND_RX + abs(v) / DIAMOND_RY < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x):
	"""
	Crea una carta rigida (clump de esferas).

	pos_x, pos_y, pos_z : centro geometrico de la carta en el mundo.
	angle_z : giro alrededor del eje Z mundial (rad), azimut en planta.
	angle_x : inclinacion alrededor del eje X mundial (rad), inclina la carta
	          hacia +/-Z (angle_x > 0 inclina el borde +Y local hacia +Z).

	La carta se genera en reposo en el plano XY local (cara en z=0, espesor en z).
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	step = 2.0 * SPHERE_RADIUS
	members = []
	x = -CARD_LENGTH / 2.0 + SPHERE_RADIUS
	while x <= CARD_LENGTH / 2.0 - SPHERE_RADIUS + 1e-9:
		y = -CARD_WIDTH / 2.0 + SPHERE_RADIUS
		while y <= CARD_WIDTH / 2.0 - SPHERE_RADIUS + 1e-9:
			local = Vector3(x, y, SPHERE_RADIUS)
			wp = rot * local + center
			members.append(
				sphere(
					wp,
					SPHERE_RADIUS,
					material=m_card,
					color=sphere_color(x + CARD_LENGTH / 2.0, y + CARD_WIDTH / 2.0),
				)
			)
			y += step
		x += step
	O.bodies.appendClumped(members)


O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1, material=m_floor, color=(0.4, 0.4, 0.4))
)

# /\ : bases separadas en X, inclinadas una hacia +Z y otra hacia -Z (se tocan arriba)
tilt = pi / 6.0
sep = CARD_WIDTH * sin(tilt) / 2.0
y_center = CARD_WIDTH / 2.0 * cos(tilt) + SPHERE_RADIUS

add_card(-sep, y_center, 0, 0, tilt)    # carta izq. inclinada hacia +Z (centro)
add_card(sep, y_center, 0, 0, -tilt)   # carta der. inclinada hacia -Z (centro)

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
