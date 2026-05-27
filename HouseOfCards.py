# -*- coding: utf-8 -*-
# Casa de cartas: N_TRIANGLES pares /\ consecutivos a lo largo de Z

from yade import utils, qt
from yade import *
from math import pi, sin, cos

# =============================================================================
# PARAMETROS
# =============================================================================
N_TRIANGLES = 3            # numero de triangulos /\ en fila

CARD_LENGTH = 63.0e-3      # eje Z (largo de la carta, profundidad)
CARD_HEIGHT = 88.0e-3      # eje Y (alto de la carta, direccion de la gravedad)
CARD_THICKNESS = 2.0e-3    # espesor fisico de referencia
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t, diametro = espesor simulado

DIAMOND_RY = 0.18 * CARD_HEIGHT   # semiejex Y del rombo en la cara
DIAMOND_RZ = 0.12 * CARD_LENGTH   # semiejex Z del rombo en la cara

TILT = pi / 6.0            # inclinacion de cada carta respecto a la vertical (30 deg)

m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.6)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.5)

# =============================================================================
# COLOR: rombo L1 (|y|/RY + |z|/RZ < 1)
# =============================================================================
def sphere_color(y_face, z_face):
	if abs(y_face) / DIAMOND_RY + abs(z_face) / DIAMOND_RZ < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


# =============================================================================
# CONSTRUCCION DE UNA CARTA (clump)
# =============================================================================
def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	pos_*   : centro de la cara interna de la carta en coordenadas mundo.
	angle_z : inclinacion alrededor de Z (rad).
	angle_x : rotacion alrededor de X (rad).
	x_local : desplazamiento del centro de esferas desde la cara interna.
	          -r = cuerpo a la izquierda (carta izquierda del /\)
	          +r = cuerpo a la derecha  (carta derecha  del /\)
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	step = 2.0 * SPHERE_RADIUS
	members = []
	y = -CARD_HEIGHT / 2.0 + SPHERE_RADIUS
	while y <= CARD_HEIGHT / 2.0 - SPHERE_RADIUS + 1e-9:
		z = -CARD_LENGTH / 2.0 + SPHERE_RADIUS
		while z <= CARD_LENGTH / 2.0 - SPHERE_RADIUS + 1e-9:
			wp = rot * Vector3(x_local, y, z) + center
			members.append(sphere(wp, SPHERE_RADIUS,
			                      material=m_card,
			                      color=sphere_color(y, z)))
			z += step
		y += step
	O.bodies.appendClumped(members)


# =============================================================================
# ESCENA
# =============================================================================
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

# Geometria del /\
sep_x = (CARD_HEIGHT / 2.0) * sin(TILT)
# y_ctr: altura del centro de la cara interna para que el borde inferior
# de las esferas quede en y = SPHERE_RADIUS (justo sobre el suelo)
y_ctr = SPHERE_RADIUS * (1.0 - sin(TILT) - cos(TILT)) + (CARD_HEIGHT / 2.0) * cos(TILT)

# Separacion entre triangulos: largo de la carta + hueco de un diametro
z_step = CARD_LENGTH + 2.0 * SPHERE_RADIUS

# Centrar la fila de triangulos en z = 0
z_start = -((N_TRIANGLES - 1) / 2.0) * z_step

for i in range(N_TRIANGLES):
	z_i = z_start + i * z_step
	add_card(-sep_x, y_ctr, z_i, -TILT, 0.0, -SPHERE_RADIUS)
	add_card( sep_x, y_ctr, z_i,  TILT, 0.0, +SPHERE_RADIUS)

# =============================================================================
# MOTORES
# =============================================================================
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
V.viewDir = (0, 0, -1)
V.upVector = (0, 1, 0)
V.center()
