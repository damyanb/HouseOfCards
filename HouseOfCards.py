# -*- coding: utf-8 -*-
# HouseOfCards - dos cartas (paralelepipedos discretizados con esferas, CSG + pack)
# Yade DEM: predicado pack.inAlignedBox + empaquetado denso de esferas
# Damyan Benjamyn Santander Huerta (reescritura esferas CSG)

from __future__ import print_function
from yade import pack, utils, qt
from yade import *
import numpy as np
from math import cos, sin, pi

# =============================================================================
# PARAMETROS GEOMETRICOS (cartas tipo naipes, algo mas gruesas que la realidad)
# =============================================================================
CARD_LENGTH = 63.0e-3          # m - lado largo de la carta (eje local x)
CARD_WIDTH = 88.0e-3           # m - lado corto en la cara (eje local y)
CARD_THICKNESS = 2.0e-3        # m - espesor fisico t de la carta
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # m - r = 2*t, una esfera en el espesor
PACK_THICKNESS = 2.0 * SPHERE_RADIUS     # m - altura del predicado = diametro 2r

# Rombo negro en la cara (fraccion del tamano de la carta, en coordenadas locales)
DIAMOND_RX = 0.12 * CARD_LENGTH
DIAMOND_RY = 0.12 * CARD_WIDTH
DIAMOND_Z_TOL = 0.55 * SPHERE_RADIUS   # capas centrales en espesor (|z| pequeno)

# Disposicion: dos cartas formando /\ (angulo 60 deg entre cartas, 30 deg a la vertical)
LEAN_FROM_VERTICAL = pi / 6.0        # rad (30 deg)
FLOOR_Y = 0.0
BASE_SEPARATION = CARD_WIDTH * sin(LEAN_FROM_VERTICAL)   # separacion de centros en x

# =============================================================================
# PARAMETROS FISICOS
# =============================================================================
RHO_CARD = 800.0               # kg/m^3  (carton)
RHO_FLOOR = 2500.0             # kg/m^3  (suelo rigido efectivo)
YOUNG_CARD = 1.0e7             # Pa
YOUNG_FLOOR = 1.0e9             # Pa
POISSON = 0.3
FRICTION_CARD = atan(0.6)      # rad  (~mu ~ 0.6 carta-carta / carta-suelo)
FRICTION_FLOOR = atan(0.5)
DAMPING = 0.35
GRAVITY = (0.0, -9.81, 0.0)

# Empaquetado
PACK_SEED = 42
PACK_RREL_FUZZ = 0.0           # todas las esferas con el mismo radio

# Colores (OpenGL)
COLOR_CARD_WHITE = (1.0, 1.0, 1.0)
COLOR_DIAMOND_BLACK = (0.0, 0.0, 0.0)

# =============================================================================
# MATERIALES
# =============================================================================
m_card = FrictMat(
	density=RHO_CARD,
	young=YOUNG_CARD,
	poisson=POISSON,
	frictionAngle=FRICTION_CARD,
)
m_floor = FrictMat(
	density=RHO_FLOOR,
	young=YOUNG_FLOOR,
	poisson=POISSON,
	frictionAngle=FRICTION_FLOOR,
)

# =============================================================================
# GEOMETRIA CSG: paralelepipedo = pack.inAlignedBox
# =============================================================================

def card_predicate():
	"""Volumen de la carta como caja alineada (CSG primitivo)."""
	hx, hy, hz = CARD_LENGTH / 2.0, CARD_WIDTH / 2.0, PACK_THICKNESS / 2.0
	return pack.inAlignedBox((-hx, -hy, -hz), (hx, hy, hz))


def pack_card_sphere_cloud():
	"""Genera esferas densas dentro del predicado CSG (no anade a O.bodies aun)."""
	pred = card_predicate()
	sp = pack.randomDensePack(
		pred,
		radius=SPHERE_RADIUS,
		rRelFuzz=PACK_RREL_FUZZ,
		material=m_card,
		returnSpherePack=True,
		seed=PACK_SEED,
	)
	return sp


def is_diamond_center(local_pos):
	"""Rombo negro en la cara central: |x|/a + |y|/b < 1 y z ~ 0."""
	x, y, z = local_pos[0], local_pos[1], local_pos[2]
	if abs(z) > DIAMOND_Z_TOL:
		return False
	if DIAMOND_RX <= 0.0 or DIAMOND_RY <= 0.0:
		return False
	return (abs(x) / DIAMOND_RX + abs(y) / DIAMOND_RY) < 1.0


def rot_matrix_axis(axis, angle):
	ax = np.asarray(axis, dtype=float)
	n = np.linalg.norm(ax)
	if n < 1e-12:
		return np.eye(3)
	ax /= n
	c, s = cos(angle), sin(angle)
	K = np.array([
		[0.0, -ax[2], ax[1]],
		[ax[2], 0.0, -ax[0]],
		[-ax[1], ax[0], 0.0],
	])
	return np.eye(3) + s * K + (1.0 - c) * np.dot(K, K)


def add_card(spheres, center, rot_mat, label):
	"""Anade esferas de una lista (pos, r) con rotacion/traslacion y color de carta."""
	n_white, n_black = 0, 0
	center = np.asarray(center, dtype=float)
	for pos, r in spheres:
		local = np.asarray(pos, dtype=float)
		world = rot_mat.dot(local) + center
		col = COLOR_DIAMOND_BLACK if is_diamond_center(local) else COLOR_CARD_WHITE
		if col == COLOR_DIAMOND_BLACK:
			n_black += 1
		else:
			n_white += 1
		O.bodies.append(sphere(tuple(world), r, material=m_card, color=col))
	print(
		'Carta %s: %d esferas (%d blancas, %d negras rombo)'
		% (label, n_white + n_black, n_white, n_black)
	)


def card_center_and_rotation(sign):
	"""
	sign = +1 carta inclinada hacia +x, sign = -1 hacia -x.
	La carta apoya con su borde inferior (y = -CARD_WIDTH/2 local) en el suelo.
	"""
	# Inclinacion en el plano y-z local, rotacion alrededor del eje x mundial
	angle = sign * LEAN_FROM_VERTICAL
	R = rot_matrix_axis((1.0, 0.0, 0.0), angle)

	# Borde inferior local y = -CARD_WIDTH/2; tras rotar, subir hasta y = FLOOR_Y + r
	edge_local = np.array([0.0, -CARD_WIDTH / 2.0, 0.0])
	edge_world = R.dot(edge_local)
	y_center = FLOOR_Y + SPHERE_RADIUS - edge_world[1]

	x_center = sign * BASE_SEPARATION / 2.0
	center = np.array([x_center, y_center, 0.0])
	return center, R

# =============================================================================
# CONSTRUCCION DE LA ESCENA
# =============================================================================
# Suelo (unica pared)
O.bodies.append(
	utils.wall(
		position=(0.0, FLOOR_Y, 0.0),
		axis=1,
		sense=1,
		material=m_floor,
		color=(0.35, 0.35, 0.4),
	)
)

print('Empaquetando carta (CSG inAlignedBox + randomDensePack)...')
sp_card = pack_card_sphere_cloud()
spheres = [(pos, r) for pos, r in sp_card]
print('Esferas por carta (aprox.):', len(spheres))

centerA, rotA = card_center_and_rotation(+1)
add_card(spheres, centerA, rotA, 'A')
centerB, rotB = card_center_and_rotation(-1)
add_card(spheres, centerB, rotB, 'B')

# =============================================================================
# MOTOR DE SIMULACION (esferas + roce)
# =============================================================================
O.engines = [
	ForceResetter(),
	InsertionSortCollider([Bo1_Sphere_Aabb(), Bo1_Wall_Aabb()]),
	InteractionLoop(
		[Ig2_Sphere_Sphere_ScGeom(), Ig2_Wall_Sphere_ScGeom()],
		[Ip2_FrictMat_FrictMat_FrictPhys()],
		[Law2_ScGeom_FrictPhys_CundallStrack()],
	),
	NewtonIntegrator(damping=DAMPING, gravity=GRAVITY),
]

O.dt = utils.PWaveTimeStep() * 0.4
O.trackEnergy = True

V = qt.View()
