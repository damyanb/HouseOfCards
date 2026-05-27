# -*- coding: utf-8 -*-
# Casa de cartas: piramide completa de N_TRIANGLES niveles

from yade import utils
from yade import *
from math import pi, sin, cos

# =============================================================================
# PARAMETROS
# =============================================================================
N_TRIANGLES = 10              # triangulos en la base; la piramide tendra N niveles

CARD_LENGTH = 63.0e-3        # lado corto (eje Z cuando la carta esta parada)
CARD_HEIGHT = 88.0e-3        # lado largo (eje Y parada, eje X tumbada = techo)
CARD_THICKNESS = 2.0e-3
SPHERE_RADIUS = 2.0 * CARD_THICKNESS   # r = 2*t

DIAMOND_RY = 0.35 * CARD_HEIGHT
DIAMOND_RZ = 0.35 * CARD_LENGTH

# Inclinacion respecto a la vertical (22.5 deg).
# Restriccion para no solapar bases con x_step = CARD_HEIGHT: TILT <= ~25.7 deg.
TILT = pi / 8.0              # 22.5 deg — triangulos agudos

m_card  = FrictMat(density=800,  young=1e7, poisson=0.3, frictionAngle=0.4)
m_floor = FrictMat(density=2500, young=1e9, poisson=0.3, frictionAngle=0.4)

# =============================================================================
# GRID DE ESFERAS CENTRADO EN (0,0)
# =============================================================================
N_Y = int(CARD_HEIGHT / (2.0 * SPHERE_RADIUS))   # 11 esferas en alto
N_Z = int(CARD_LENGTH / (2.0 * SPHERE_RADIUS))   # 7  esferas en largo
STEP = 2.0 * SPHERE_RADIUS

# =============================================================================
# GEOMETRIA DERIVADA
# =============================================================================
h_half = (N_Y - 1) * SPHERE_RADIUS          # semialtura del grid (40 mm)
sep_x  = (CARD_HEIGHT / 2.0) * sin(TILT)   # separacion horizontal carta-eje

# Separacion entre centros de triangulos vecinos = lado largo de la carta techo.
# Con TILT=22.5 deg el hueco entre bases adyacentes es ~8 mm (sin solapamiento).
x_step = CARD_HEIGHT

# =============================================================================
# PARAMETROS DE GRABACION
# =============================================================================
RECORD_DT     = 0.1   # intervalo de tiempo fisico entre grabaciones (s)
ENERGY_THRESH = 0.1   # umbral de energia cinetica para detener la grabacion (J)

# =============================================================================
# FUNCIONES
# =============================================================================
def sphere_color(y, z):
	if abs(y) / DIAMOND_RY + abs(z) / DIAMOND_RZ < 1.0:
		return (0, 0, 0)
	return (1, 1, 1)


def add_card(pos_x, pos_y, pos_z, angle_z, angle_x, x_local):
	"""
	Crea una carta rigida (clump de esferas).
	Retorna los IDs de las 4 esferas esquina de la carta:
	  [esq(iy=0,iz=0), esq(iy=0,iz=N_Z-1),
	   esq(iy=N_Y-1,iz=0), esq(iy=N_Y-1,iz=N_Z-1)]

	El orden en 'members' es outer-iy / inner-iz, de modo que:
	  indice en members = iy*N_Z + iz
	Tras appendClumped, ids[0] = clump, ids[1:] = esferas en el mismo orden.
	"""
	center = Vector3(pos_x, pos_y, pos_z)
	rot = Quaternion((0, 0, 1), angle_z) * Quaternion((1, 0, 0), angle_x)
	members = []
	for iy in range(N_Y):
		y = (iy - (N_Y - 1) / 2.0) * STEP
		for iz in range(N_Z):
			z = (iz - (N_Z - 1) / 2.0) * STEP
			wp = rot * Vector3(x_local, y, z) + center
			members.append(sphere(wp, SPHERE_RADIUS,
			                      material=m_card,
			                      color=sphere_color(y, z)))
	# En Yade 2018 appendClumped retorna (clumpId, [memberIds])
	ids = O.bodies.appendClumped(members)
	m = ids[1]   # lista de IDs de esferas miembro
	return [m[0 * N_Z + 0],
	        m[0 * N_Z + (N_Z - 1)],
	        m[(N_Y - 1) * N_Z + 0],
	        m[(N_Y - 1) * N_Z + (N_Z - 1)]]


# =============================================================================
# RECURRENCIA DE ALTURAS POR NIVEL
# =============================================================================
y_ctr_0 = SPHERE_RADIUS * (1.0 - sin(TILT)) + h_half * cos(TILT)

level_y_ctr  = []   # altura y_ctr para triangulos de cada nivel
level_y_roof = []   # altura y de la cara interna del techo de cada nivel

y_ctr_k = y_ctr_0
for k in range(N_TRIANGLES):
	level_y_ctr.append(y_ctr_k)
	y_peak_top_k = y_ctr_k + SPHERE_RADIUS * (1.0 + sin(TILT)) + h_half * cos(TILT)
	level_y_roof.append(y_peak_top_k)
	y_ctr_k = y_ctr_k + 4.0 * SPHERE_RADIUS + 2.0 * h_half * cos(TILT)

# =============================================================================
# ESCENA
# =============================================================================
O.bodies.append(
	utils.wall(position=(0, 0, 0), axis=1, sense=1,
	           material=m_floor, color=(0.4, 0.4, 0.4))
)

x_start = -((N_TRIANGLES - 1) / 2.0) * x_step

corner_ids = []   # IDs de las 4 esferas esquina de cada carta

for k in range(N_TRIANGLES):
	n_tri  = N_TRIANGLES - k
	n_roof = n_tri - 1
	y_tri  = level_y_ctr[k]
	y_roof = level_y_roof[k]
	x_orig = x_start + k * (x_step / 2.0)

	for i in range(n_tri):
		xi = x_orig + i * x_step
		corner_ids += add_card(xi - sep_x, y_tri, 0.0, -TILT, 0.0, -SPHERE_RADIUS)
		corner_ids += add_card(xi + sep_x, y_tri, 0.0,  TILT, 0.0, +SPHERE_RADIUS)

	for i in range(n_roof):
		xi    = x_orig + i * x_step
		x_mid = xi + x_step / 2.0
		corner_ids += add_card(x_mid, y_roof, 0.0, pi / 2.0, 0.0, SPHERE_RADIUS)

# Carta extra
y_apex  = level_y_roof[N_TRIANGLES - 1]
y_extra = y_apex + 5.0 * CARD_HEIGHT

corner_ids += add_card(0.1 * CARD_LENGTH, y_extra, 0.1 * CARD_LENGTH,
                       pi / 3.0, pi / 5.0, SPHERE_RADIUS)

# =============================================================================
# GRABACION DE ESQUINAS
# =============================================================================
_rec_file   = open('corner_positions.csv', 'w')
_rec_file.write('t,sphere_id,x,y,z\n')
_rec_last_t = [-RECORD_DT]
_rec_active = [True]

def record_corners():
	if not _rec_active[0]:
		return
	t = O.time
	if t - _rec_last_t[0] >= RECORD_DT - 1e-12:
		_rec_last_t[0] = t
		for sid in corner_ids:
			p = O.bodies[sid].state.pos
			_rec_file.write('%g,%d,%.6e,%.6e,%.6e\n' % (t, sid, p[0], p[1], p[2]))
		_rec_file.flush()
	if utils.kineticEnergy() < ENERGY_THRESH:
		_rec_file.close()
		_rec_active[0] = False
		O.pause()

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
	PyRunner(command='record_corners()', iterPeriod=50),
]
O.dt = utils.PWaveTimeStep() * 0.4

O.run(wait=True)
