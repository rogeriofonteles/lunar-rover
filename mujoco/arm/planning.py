"""Seeded RRT-Connect, approximate collision geometry, and timed joint paths."""
from dataclasses import dataclass
import json
from pathlib import Path
import time

import mujoco
import numpy as np

from .buttons import BUTTON_NAMES, button_target
from .model import positive, positive_int, vector

SCENE_PATH = Path(__file__).with_name("planning_scene.json")


def point_segment_distance(point, a, b):
    delta = b-a
    t = np.clip(np.dot(point-a, delta)/max(np.dot(delta, delta), 1e-20), 0, 1)
    return float(np.linalg.norm(point-a-t*delta))


def segment_distance(a, b, c, d):
    u, v, w = b-a, d-c, a-c
    A, B, C, D, E = u@u, u@v, v@v, u@w, v@w
    distances = [point_segment_distance(a,c,d), point_segment_distance(b,c,d),
                 point_segment_distance(c,a,b), point_segment_distance(d,a,b)]
    det = A*C-B*B
    if det > 1e-16:
        s, t = (B*E-C*D)/det, (A*E-B*D)/det
        if 0 <= s <= 1 and 0 <= t <= 1:
            distances.append(float(np.linalg.norm(w+s*u-t*v)))
    return min(distances)


class CollisionWorld:
    """Capsules for moving links, spheres/boxes for selected scene geometry.

    Decorative bolts, cables, button stands and switch contact are excluded.
    Self checks exclude nearby links sharing a wrist/shoulder assembly.
    Edge checks are discrete, not continuous collision certification.
    """
    def __init__(self, arm, scene=None):
        self.arm = arm
        self.data = mujoco.MjData(arm.model)
        self.scene = json.loads(SCENE_PATH.read_text()) if scene is None else scene
        self.lower = vector(self.scene["joint_lower"], 6, "joint_lower")
        self.upper = vector(self.scene["joint_upper"], 6, "joint_upper")
        if np.any(self.lower >= self.upper):
            raise ValueError("Invalid planning joint bounds")
        self.radii = np.asarray(self.scene["link_radii"])
        self.margin = positive(self.scene["margin_m"], "margin_m")
        self.resolution = positive(self.scene["edge_resolution_rad"], "edge_resolution_rad")
        self.checks = 0
        self.body_ids = [arm.model.body(n).id for n in
                         ("shoulder", "elbow", "wrist_roll", "wrist_pitch", "tool_roll")]

    def points(self, q):
        self.data.qpos[:] = q
        mujoco.mj_kinematics(self.arm.model, self.data)
        return np.vstack((self.data.xpos[self.body_ids], self.data.site("button_tip").xpos))

    def clearance(self, q):
        self.checks += 1
        if not np.isfinite(q).all() or np.any(q < self.lower) or np.any(q > self.upper):
            return -np.inf
        points = self.points(q)
        clearance = float(np.min(points[:, 2])-max(self.radii))  # floor
        for i, radius in enumerate(self.radii):
            a,b = points[i:i+2]
            for obstacle in self.scene["spheres"]:
                clearance = min(clearance, point_segment_distance(np.array(obstacle["center"]),a,b)
                                -radius-obstacle["radius"])
            # Distance to AABBs sampled with a conservative Lipschitz correction.
            # The true segment clearance is >= min(sample distances)-spacing/2.
            count = max(2, int(np.ceil(np.linalg.norm(b-a)/.025))+1)
            samples = np.linspace(a,b,count)
            correction = np.linalg.norm(b-a)/(count-1)/2
            for box in self.scene["boxes"]:
                outside = np.maximum(np.abs(samples-box["center"])-box["half_size"],0)
                clearance = min(clearance, float(np.min(np.linalg.norm(outside,axis=1)))-radius-correction)
            for j in range(i+3,len(self.radii)):
                clearance = min(clearance, segment_distance(a,b,points[j],points[j+1])-radius-self.radii[j])
        return clearance

    def valid(self, q):
        return self.clearance(q) >= self.margin

    def edge(self, a, b):
        count = max(1, int(np.ceil(np.max(np.abs(b-a))/self.resolution)))
        return all(self.valid(q) for q in np.linspace(a,b,count+1))


def rrt_connect(start, goal, valid, edge, lower, upper, seed=6, step=.25, max_iterations=6000):
    """Bidirectional joint-space RRT with greedy connection and explicit failure."""
    step = positive(step,"rrt_step")
    max_iterations = positive_int(max_iterations,"rrt_iterations")
    start, goal = np.asarray(start,float), np.asarray(goal,float)
    if not valid(start) or not valid(goal):
        raise ValueError("Início ou destino inválido no modelo de colisão")
    rng = np.random.default_rng(seed)
    trees = [([start.copy()],[-1]),([goal.copy()],[-1])]

    def extend(tree, target):
        nodes, parents = tree
        nearest = int(np.argmin(np.linalg.norm(np.asarray(nodes)-target,axis=1)))
        delta = target-nodes[nearest]
        distance = np.linalg.norm(delta)
        if distance < 1e-12:
            return nearest, True
        q = nodes[nearest]+delta*min(1.,step/distance)
        if not edge(nodes[nearest],q):
            return None, False
        nodes.append(q); parents.append(nearest)
        return len(nodes)-1, distance <= step

    def ancestry(tree,index):
        path=[]
        while index != -1:
            path.append(tree[0][index]); index=tree[1][index]
        return path[::-1]

    for iteration in range(max_iterations):
        side = iteration % 2
        sample = trees[1-side][0][0] if rng.random()<.15 else rng.uniform(lower,upper)
        first,_ = extend(trees[side],sample)
        if first is None:
            continue
        while True:
            second,reached = extend(trees[1-side],trees[side][0][first])
            if second is None:
                break
            if reached:
                a,b = ancestry(trees[side],first),ancestry(trees[1-side],second)
                path = a + b[-2::-1]
                if side:
                    path = path[::-1]
                return np.asarray(path), {"iterations":iteration+1,"nodes":sum(len(t[0]) for t in trees)}
    raise RuntimeError(f"RRT-Connect não encontrou caminho após {max_iterations} iterações")


def shortcut(path, edge, seed=6, attempts=150):
    path = list(path.copy())
    rng=np.random.default_rng(seed)
    for _ in range(attempts):
        if len(path)<3:
            break
        a,b=sorted(rng.choice(len(path),2,replace=False))
        if b>a+1 and edge(path[a],path[b]):
            path[a+1:b]=[]
    return np.asarray(path)


@dataclass
class Route:
    states: np.ndarray
    accelerations: np.ndarray
    arrivals: list
    paths: list
    planning: list
    sequence: list


def plan_route(arm, sequence=("red","yellow","green","red"), seed=6,
               speed=.65, acceleration=1.2, dwell=.6):
    if len(sequence)<2 or any(name not in BUTTON_NAMES for name in sequence):
        raise ValueError("A sequência precisa de pelo menos dois botões válidos")
    speed,acceleration,dwell = (positive(speed,"speed"),positive(acceleration,"acceleration"),positive(dwell,"dwell"))
    world=CollisionWorld(arm)
    states=[np.r_[button_target(sequence[0])["joint_positions_rad"],np.zeros(6)]]
    accelerations=[np.zeros(6)]
    arrivals=[{"button":sequence[0],"index":0}]
    paths,stats=[],[]
    for leg,(source,dest) in enumerate(zip(sequence,sequence[1:])):
        a=np.array(button_target(source)["joint_positions_rad"])
        b=np.array(button_target(dest)["joint_positions_rad"])
        checks=world.checks; start=time.perf_counter()
        direct=world.edge(a,b)
        raw,info=rrt_connect(a,b,world.valid,world.edge,world.lower,world.upper,seed=seed+leg)
        path=shortcut(raw,world.edge,seed=seed+leg)
        info.update(source=source,destination=dest,direct_valid=direct,raw_waypoints=len(raw),
                    waypoints=len(path),checks=world.checks-checks,seconds=time.perf_counter()-start,
                    length_rad=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()))
        stats.append(info);paths.append(path)
        # Rest-to-rest quintics on every validated edge stay on that edge.
        # Stopping at corners preserves geometry (unvalidated splines can cut obstacles).
        for qa,qb in zip(path,path[1:]):
            delta=qb-qa; distance=np.max(np.abs(delta))
            duration=max(.4,1.875*distance/speed,np.sqrt(5.774*distance/acceleration))
            count=int(np.ceil(duration/arm.dt)); duration=count*arm.dt
            for k in range(1,count+1):
                s=k/count
                h=10*s**3-15*s**4+6*s**5
                hd=(30*s**2-60*s**3+30*s**4)/duration
                hdd=(60*s-180*s**2+120*s**3)/duration**2
                states.append(np.r_[qa+h*delta,hd*delta]);accelerations.append(hdd*delta)
        arrivals.append({"button":dest,"index":len(states)-1})
        for _ in range(int(np.ceil(dwell/arm.dt))):
            states.append(np.r_[b,np.zeros(6)]);accelerations.append(np.zeros(6))
    return Route(np.asarray(states),np.asarray(accelerations),arrivals,paths,stats,list(sequence)),world


def show_planning_obstacles(model):
    """Enable the route-only obstacle already included in the generated MJCF."""
    scene=json.loads(SCENE_PATH.read_text())
    for obstacle in scene["spheres"]:
        model.geom_rgba[model.geom(obstacle["name"]).id,3]=1.
