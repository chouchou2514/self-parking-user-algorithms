"""학생 자율주차 알고리즘 스켈레톤 모듈.

이 파일만 수정하면 되고, 네트워킹/IPC 관련 코드는 `ipc_client.py`에서
자동으로 처리합니다. 학생은 아래 `PlannerSkeleton` 클래스나 `planner_step`
함수를 원하는 로직으로 교체/확장하면 됩니다.
"""

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

#ADD
import math #gives me access to math.atan2, math.sin, math.pi which Pure Pursuit needs to calculate angles
from dataclasses import field


#ADD
def normalize_angle(angle: float) -> float:
    '''
    Angles in Python can exceed 360°.
    This function always returns them to between -180° and +180° (in radians).
    Pure Pursuit needs this to avoid running in the wrong direction.
    '''
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

#ADD
def distance(p1, p2) -> float:
    '''
     Calculate the distance between two points (x1,y1) and (x2,y2).
     This is the Pythagorean theorem: square root of (dx² + dy²).
     We use it to determine if we are close to a waypoint.
    '''
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

#ADD
'''
Class Control to control the movment of the car'''
class PurePursuitController:

    def __init__(self, look_ahead: float = 2.5, k_v: float=0.3):
        self.look_ahead = look_ahead #distance à laquelle la voiture vise devant elle
        self.k_v = k_v #coeff qui adapte le look_ahead à la vitesse

    def compute_steer(self, x, y, yaw, waypoints, v, wheelbase, max_steer):
        '''
        La fonction qui calcule l'angle du volant

        :param x:position actuelle de la voiture
        :param y:position actuelle de la voiture
        :param yaw:direction actuelle (angle en radians)
        :param waypoints:la liste des points du chemin
        :param v:vitesse actuelle
        :param wheelbase: longueur de la voiture
        :param max_steer:angle maximum du volant autorisé par le simulateur
        :return:
        '''
        Ld = max(self.look_ahead, self.k_v + abs(v))
        '''
        Calcule la distance look-ahead. 
        max prend le plus grand des deux valeurs — soit 2.5m minimum, 
        soit 0.3 * vitesse si la voiture va vite. 
        abs(v) c'est la valeur absolue de la vitesse (toujours positif).
        '''

        target = waypoints[-1][:2]
        '''
        Par défaut, le point cible c'est le dernier waypoint. 
        Les waypoints sont une liste qui ressemble à ca:
        waypoints = [
        (10.0, 5.0, 0.0),   # point 0 : x=10, y=5,  yaw=0.0
        (20.0, 8.0, 0.1),   # point 1 : x=20, y=8,  yaw=0.1
        (30.0, 6.0, 0.2),   # point 2 : x=30, y=6,  yaw=0.2
        ]
        [-1] en Python veut dire "le dernier élément". 
        [:2] veut dire "prends seulement les deux premières valeurs" 
        (x et y, pas le yaw).
        '''
        for wp in waypoints:
            if distance((x, y), (wp[0], wp[1])) >= Ld:
                target = (wp[0], wp[1])
                break
        '''
        On parcourt tous les waypoints un par un. 
        Dès qu'on en trouve un qui est à plus de Ld mètres de nous, 
        on le choisit comme point cible et on arrête (break).
        '''

        dx = target[0] - x
        dy = target[1] - y
        '''
        On calcule la différence de position entre nous et le point cible.
        '''

        alpha = normalize_angle(math.atan2(dy, dx) - yaw)
        '''
        math.atan2(dy, dx) calcule l'angle vers le point cible. 
        atan2 c'est l'inverse de Pythagore.
        Pythagore : tu donnes dx et dy → il te donne la distance
        atan2 : tu donnes dy et dx → il te donne l'angle
        On soustrait yaw (notre direction actuelle) pour avoir l'angle relatif 
        —> c'est à dire de combien on doit tourner. 
        normalize_angle ramène ça entre -180° et +180°.
        '''

        steer = math.atan2(2.0 * wheelbase * math.sin(alpha), Ld)
        '''
        La formule Pure Pursuit. 
        Elle transforme alpha en angle de volant.
        '''

        steer = max(-max_steer, min(max_steer, steer))
        '''
        On s'assure que le volant ne dépasse pas les limites du simulateur. 
        Puis on renvoie la valeur. max(-max_steer, min(max_steer, steer)) 
        c'est juste une façon de "clipper" entre -0.61 et +0.61. (valeurs viennent du README)
        '''
        return steer


#ADD
#compute_longitudinal : This is the function that controls speed — accelerating or braking

def compute_longitudinal(v, target_v, max_accel, max_brake):

    error = target_v - v
    '''
     la différence entre la vitesse qu'on veut et la vitesse actuelle. 
     Si on veut 1.5 m/s et qu'on roule à 1.0 m/s, error = 0.5.
    '''

    if error > 0.05:
        '''
        on est trop lent → on accélère. 
        Le 0.05 c'est une petite marge pour éviter de corriger 
        en permanence pour rien.
        '''
        return min(0.4 * error, max_accel), 0.0
        '''
        on accélère proportionnellement à l'écart, 
        mais jamais plus que le max autorisé. 
        Le 0.0 c'est le brake (on ne freine pas en même temps).
        '''


    elif error < -0.05: #on est trop rapide → on freine
        return 0.0, min(0.5*abs(error), max_brake)

    else:
        return 0.0, 0.0 #on est à la bonne vitesse → ni accel ni brake






def pretty_print_map_summary(map_payload: Dict[str, Any]) -> None:
    extent = map_payload.get("extent") or [None, None, None, None]
    slots = map_payload.get("slots") or []
    occupied = map_payload.get("occupied_idx") or []
    free_slots = len(slots) - sum(1 for v in occupied if v)
    print("[algo] map extent :", extent)
    print("[algo] total slots:", len(slots), "/ free:", free_slots)
    stationary = map_payload.get("grid", {}).get("stationary")
    if stationary:
        rows = len(stationary)
        cols = len(stationary[0]) if stationary else 0
        print("[algo] grid size  :", rows, "x", cols)


@dataclass
class PlannerSkeleton:
    """경로 계획/제어 로직을 담는 기본 스켈레톤 클래스입니다."""

    map_data: Optional[Dict[str, Any]] = None
    map_extent: Optional[Tuple[float, float, float, float]] = None
    cell_size: float = 0.5
    stationary_grid: Optional[List[List[float]]] = None
    waypoints: List[Tuple[float, float]] = None

    def __post_init__(self) -> None:
        if self.waypoints is None:
            self.waypoints = []
























    def set_map(self, map_payload: Dict[str, Any]) -> None:
        """시뮬레이터에서 전송한 정적 맵 데이터를 보관합니다."""

        self.map_data = map_payload
        self.map_extent = tuple(
            map(float, map_payload.get("extent", (0.0, 0.0, 0.0, 0.0)))
        )
        self.cell_size = float(map_payload.get("cellSize", 0.5))
        self.stationary_grid = map_payload.get("grid", {}).get("stationary")
        pretty_print_map_summary(map_payload)
        self.waypoints.clear()

    def compute_path(self, obs: Dict[str, Any]) -> None:
        """관측과 맵을 이용해 경로(웨이포인트)를 준비합니다."""

        # TODO: A*, RRT*, Hybrid A* 등으로 self.waypoints를 채우세요.
        self.waypoints.clear()

    def compute_control(self, obs: Dict[str, Any]) -> Dict[str, float]:
        """경로를 따라가기 위한 조향/가감속 명령을 산출합니다."""

        '''
        #Code original du projet à modifier
        # 예시: 기본 데모 제어. 학생은 원하는 알고리즘으로 대체하면 됩니다.
        t = float(obs.get("t", 0.0)) #récupère le temps écoulé depuis le début
        v = float(obs.get("state", {}).get("v", 0.0)) #récupère la vitesse actuelle

        cmd = {"steer": 0.0, "accel": 0.0, "brake": 0.0, "gear": "D"}

        if t < 2.0:
            cmd["accel"] = 0.6 #entre 0 et 2sec : accel à 0.6
        elif t < 3.0:
            cmd["brake"] = 0.3 #entre 2 et 3 sec : freine a 0.3
        else:
            cmd["steer"] = 0.07 #après 3 sec : tourne le volant à 0.07 et accèlère un peu si trop lent
            if v < 1.0:
                cmd["accel"] = 0.2

        return cmd
        '''

        state = obs.get("state", {})

        '''
        obs c'est le paquet que le simulateur envoie à chaque frame. 
        On extrait la partie "state" qui contient la position de la voiture. 
        Le {} c'est la valeur par défaut si "state" n'existe pas.
        '''
        x = float(state.get("x", 0.0))
        y = float(state.get("y", 0.0))
        yaw = float(state.get("yaw", 0.0))
        v = float(state.get("v", 0.0))
        '''
        n récupère les 4 informations de la voiture depuis le simulateur :
        x, y → position sur la carte
        yaw → direction (en radians)
        v → vitesse actuelle
        float() convertit en nombre décimal au cas où le simulateur envoie autre chose.
        '''

        limits = obs.get("limits", {})
        wheelbase = float(limits.get("L", 2.6))
        max_steer = float(limits.get("maxSteer", 0.6109))
        max_accel = float(limits.get("maxAccel", 3.0))
        max_brake = float(limits.get("maxBrake", 7.0))
        '''
        On récupère les limites physiques de la voiture 
        envoyées par le simulateur:
        longueur, angle max du volant, accélération max, freinage max. 
        Les valeurs après la virgule sont les défauts 
        si le simulateur ne les envoie pas.
        '''


        # Pas de waypoints → on attend le path planning
        if not self.waypoints:
            return {"steer": 0.0, "accel": 0.0, "brake": 0.0, "gear": "D"}

        # Pure Pursuit pour le volant
        controller = PurePursuitController() #Création de l'instance de ma class"
        steer = controller.compute_steer(
            x, y, yaw, self.waypoints, v, wheelbase, max_steer
        )
        #steer = direction
        #compute_steer récupère l'angle du volant gràace à toutes les infos sur la voiture


        target_v = 1.5 #La vitesse cible par défaut : 1.5 m/s

        target_slot = obs.get("target_slot", [])
        if target_slot:
            cx = (target_slot[0] + target_slot[1]) / 2.0
            cy = (target_slot[2] + target_slot[3]) / 2.0
            '''
            On récupère la place de parking cible. 
            target_slot c'est [x_min, x_max, y_min, y_max]. 
            On calcule le centre de la place en faisant 
            la moyenne de x_min et x_max, pareil pour y.
            '''

            dist_to_goal = distance((x, y), (cx, cy))
            if dist_to_goal < 3.0:
                target_v = max(0.3, 1.5 * dist_to_goal / 3.0)
            '''
            On calcule la distance entre la voiture et le centre de la place.
            Si on est à moins de 3 mètres, on ralentit progressivement. 
            À 3m → 1.5 m/s, 
            à 1m → 0.5 m/s, 
            à 0m → 0.3 m/s minimum pour ne pas s'arrêter trop tôt.
            '''

        accel, brake = compute_longitudinal(v, target_v, max_accel, max_brake)
        '''
        On appelle ta fonction compute_longitudinal 
        avec la vitesse actuelle et la vitesse cible. 
        Elle retourne accel et brake.
        '''

        return {"steer": float(steer), "accel": float(accel),
                "brake": float(brake), "gear": "D"}
        '''
        On renvoie les 4 commandes au simulateur. 
        "gear": "D" c'est toujours Drive — on avance toujours.
        '''




# 전역 planner 인스턴스 (통신 모듈이 이 객체를 사용합니다.)
planner = PlannerSkeleton()


def handle_map_payload(map_payload: Dict[str, Any]) -> None:
    """통신 모듈에서 맵 패킷을 받을 때 호출됩니다."""

    planner.set_map(map_payload)


def planner_step(obs: Dict[str, Any]) -> Dict[str, Any]:
    """통신 모듈에서 매 스텝 호출하여 명령을 생성합니다."""

    try:
        return planner.compute_control(obs)
    except Exception as exc:
        print(f"[algo] planner_step error: {exc}")
        return {"steer": 0.0, "accel": 0.0, "brake": 0.5, "gear": "D"}

