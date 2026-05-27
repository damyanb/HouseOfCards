# -*- coding: utf-8 -*-
# Dos cartas rigidas (clump) - /\ con gravedad en -Y, espesor en X

from yade import utils, qt
from yade import *
from math import pi, sin, cos, sqrt

# --- geometria (m) ---
CARD_LENGTH = 63.0e-3      # eje Z (profundidad, borde largo)
CARD_HEIGHT = 88.0e-3      # eje Y (altura de la carta parada)
CARD_THICKNESS = 2.0e-3    # eje X (espesor, una sola capa de esferas)
SPHERE_RADIUS = CARD_THICKNESS   # r = t, diametro = 2t = espesor
DIAMOND_RZ = 0.10 * CARD_LENGTH
DIAMOND_RY = 0.10 * CARD_HEIGHT

# --- fisica ---
m_card = FrictMat(density=800, young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)


def sphere_color(y_face, z_face):
	# rombo en la cara YZ de la carta
	cy = y_face / DIAMOND_RY
	cz = z_face / DIAMOND_RZ
	if abs(cy + cz) + abs(cy - cz) < 2.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x):
	"""
	Carta rigida (clump de esferas).

	Marco local de la carta en reposo:
	  X: espesor (una esfera de diametro, centro en x=0)
	  Y: altura  (de -CARD_HEIGHT/2 a +CARD_HEIGHT/2)
	  Z: largo   (de -CARD_LENGTH/2 a +CARD_LENGTH/2)

	angle_z: inclinacion en el plano XY (rad).
	         >0 inclina el techo hacia -X, <0 hacia +X.
	angle_x: rotacion en el plano YZ (rad, normalmente 0).

	pos_* : centro geometrico de la carta en el mundo.
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	step = 2.0 * SPHERE_RADIUS
	members = []
	# cara en el plano YZ, espesor en X (x=0)
	y = -CARD_HEIGHT / 2.0 + SPHERE_RADIUS
	while y <= CARD_HEIGHT / 2.0 - SPHERE_RADIUS + 1e-9:
		z = -CARD_LENGTH / 2.0 + SPHERE_RADIUS
		while z <= CARD_LENGTH / 2.0 - SPHERE_RADIUS + 1e-9:
			local = Vector3(0, y, z)
			wp = rot * local + center
			members.append(
				sphere(
					wp,
					SPHERE_RADIUS,
					material=m_card,
					color=sphere_color(y, z),
				)
			)
			z += step
		y += step
	O.bodies.appendClumped(members)


# suelo en Y=0, gravedad en -Y
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1, material=m_floor, color=(0.4, 0.4, 0.4))
)

# /\ : inclinacion alrededor de Z
# carta izq. en -X, inclinada -tilt (techo hacia +X = centro)
# carta der. en +X, inclinada +tilt (techo hacia -X = centro)
tilt   = pi / 6.0
sep_x  = (CARD_HEIGHT / 2.0) * sin(tilt)
y_ctr  = SPHERE_RADIUS + (CARD_HEIGHT / 2.0) * cos(tilt)

add_card(-sep_x, y_ctr, 0.0, -tilt, 0.0)
add_card( sep_x, y_ctr, 0.0,  tilt, 0.0)

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
V.viewDir = (0, 0, -1)   # mirar desde +Z hacia el origen
V.upVector = (0, 1, 0)   # Y arriba
V.center()
