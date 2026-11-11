"""Mechanical illustration of the Acrobot; adds no mass or contact geometry.

Keep Drake's AcrobotGeometry pose output so the artwork follows the same
joint axes and link lengths as AcrobotPlant, including recorded animations.
"""

import math

from pydrake.common import MemoryFile
from pydrake.examples import AcrobotGeometry
from pydrake.geometry import (
    Box, Cylinder, GeometryInstance, InMemoryMesh,
    MakePhongIllustrationProperties, Mesh, Meshcat, Role,
)
from pydrake.math import RigidTransform, RotationMatrix


ALUMINUM = (0.64, 0.69, 0.74, 1.0)
GRAPHITE = (0.12, 0.15, 0.19, 1.0)
BLACK = (0.035, 0.045, 0.06, 1.0)
ORANGE = (0.96, 0.29, 0.065, 1.0)
STEEL = (0.36, 0.41, 0.47, 1.0)
BLUE = (0.12, 0.65, 0.85, 1.0)
AXIS_Y = RotationMatrix.MakeXRotation(math.pi / 2)


def _slotted_plate(length, radius=0.095, thickness=0.022):
    """Extrude a rounded plate with a real through-slot, in the x-z plane."""
    def outline(top, bottom, r):
        points = []
        for center, start in ((top, 0), (bottom, math.pi)):
            for index in range(17):
                angle = start + math.pi * index / 16
                points.append((r * math.cos(angle), center + r * math.sin(angle)))
        return points

    outside = outline(0, -length, radius)
    inside = outline(-0.19, -length + 0.19, 0.038)
    count = len(outside)
    vertices = [(x, y, z) for y in (-thickness / 2, thickness / 2)
                for loop in (outside, inside) for x, z in loop]
    faces = []
    for i in range(count):
        j = (i + 1) % count
        # Front and back annulus, outside wall, and inside slot wall.
        faces.extend(((i, j, count + j, count + i),
                      (2 * count + i, 3 * count + i, 3 * count + j, 2 * count + j),
                      (i, 2 * count + i, 2 * count + j, j),
                      (count + i, count + j, 3 * count + j, 3 * count + i)))
    obj = "\n".join(f"v {x} {y} {z}" for x, y, z in vertices)
    # Triangles also work with renderers that do not support polygon faces.
    for a, b, c, d in faces:
        obj += f"\nf {a + 1} {b + 1} {c + 1}\nf {a + 1} {c + 1} {d + 1}"
    return Mesh(InMemoryMesh(mesh_file=MemoryFile(obj, ".obj", "acrobot_link")))


class _Parts:
    def __init__(self, scene_graph, source, frame=None):
        self.scene_graph, self.source, self.frame = scene_graph, source, frame

    def add(self, name, shape, xyz, color, rotation=None):
        pose = (RigidTransform(xyz) if rotation is None
                else RigidTransform(rotation, xyz))
        instance = GeometryInstance(pose, shape, name)
        instance.set_illustration_properties(MakePhongIllustrationProperties(color))
        if self.frame is None:
            self.scene_graph.RegisterAnchoredGeometry(self.source, instance)
        else:
            self.scene_graph.RegisterGeometry(self.source, self.frame, instance)

    def box(self, name, size, xyz, color):
        self.add(name, Box(*size), xyz, color)

    def cylinder(self, name, radius, depth, xyz, color, rotation=AXIS_Y):
        self.add(name, Cylinder(radius, depth), xyz, color, rotation)

    def bolts(self, name, radius, y, z=0, count=6, bolt_radius=0.009):
        for index in range(count):
            angle = 2 * math.pi * index / count + math.pi / 6
            x, height = radius * math.cos(angle), z + radius * math.sin(angle)
            self.cylinder(f"{name}_{index}_washer", bolt_radius * 1.5, 0.004,
                          [x, y, height], STEEL)
            self.cylinder(f"{name}_{index}_head", bolt_radius, 0.009,
                          [x, y - 0.005, height], ALUMINUM)
            self.cylinder(f"{name}_{index}_socket", bolt_radius * 0.46, 0.001,
                          [x, y - 0.010, height], BLACK)


def _link(parts, length, y, color):
    plate = _slotted_plate(length)
    for side, offset in (("front", -0.052), ("rear", 0.052)):
        parts.add(f"{side}_machined_plate", plate, [0, y + offset, 0], color)
    # Metal spacers join the two plates without filling the lightening slot.
    for index, z in enumerate((-0.14, -length / 2, -length + 0.14)):
        parts.cylinder(f"cross_spacer_{index}", 0.018, 0.12, [0, y, z], ALUMINUM)
        parts.cylinder(f"spacer_bolt_{index}", 0.011, 0.008,
                       [0, y - 0.069, z], BLACK)
    # Slim edge strips and an inset identification panel on the solid web.
    for x in (-0.079, 0.079):
        parts.box(f"edge_rail_{x}", [0.012, 0.023, length - 0.20],
                  [x, y - 0.053, -length / 2], ALUMINUM)
    parts.box("identification_plate", [0.05, 0.003, 0.085],
              [0, y - 0.065, -0.10], GRAPHITE)
    for index in range(3):
        parts.box(f"identification_mark_{index}", [0.032, 0.002, 0.004],
                  [0, y - 0.068, -0.077 - index * 0.014], ALUMINUM)


def add_robot_geometry(builder, plant, scene_graph):
    """Replace the example's illustration, retaining its kinematic system."""
    context = plant.CreateDefaultContext()
    params = plant.get_parameters(context)
    previous_sources = set(scene_graph.model_inspector().GetAllSourceIds())
    geometry = AcrobotGeometry.AddToBuilder(
        builder, plant.get_output_port(0), params, scene_graph)
    inspector = scene_graph.model_inspector()
    source, = set(inspector.GetAllSourceIds()) - previous_sources
    frames = {inspector.GetName(frame): frame for frame in inspector.FramesForSource(source)}
    # Some Drake Python releases omit AcrobotParams.l2(); the stock lower
    # cylinder still carries the exact length selected by AcrobotGeometry.
    lower_geometry, = inspector.GetGeometries(frames["lower_link"], Role.kIllustration)
    l1, l2 = params.l1(), inspector.GetShape(lower_geometry).length()
    for frame in (scene_graph.world_frame_id(), *frames.values()):
        scene_graph.RemoveRole(source, frame, Role.kIllustration)

    fixed = _Parts(scene_graph, source)
    upper = _Parts(scene_graph, source, frames["upper_link"])
    lower = _Parts(scene_graph, source, frames["lower_link"])

    # A bench-edge fixture leaves the moving links in front of the support.
    fixed.box("mounting_base", [0.92, 0.78, 0.075], [0, 0.44, -0.54], GRAPHITE)
    fixed.box("base_top", [0.84, 0.70, 0.018], [0, 0.44, -0.494], ALUMINUM)
    for x in (-0.35, 0.35):
        for y in (0.16, 0.71):
            fixed.cylinder(f"anchor_{x}_{y}", 0.026, 0.012,
                           [x, y, -0.479], BLACK, RotationMatrix())
    fixed.box("pedestal", [0.25, 0.27, 0.49], [0, 0.42, -0.24], GRAPHITE)
    fixed.box("pedestal_face", [0.20, 0.008, 0.40], [0, 0.28, -0.23], ALUMINUM)
    fixed.box("bearing_bracket", [0.38, 0.22, 0.32], [0, 0.20, 0], GRAPHITE)
    fixed.cylinder("shoulder_bearing_housing", 0.165, 0.22, [0, 0.055, 0], ALUMINUM)
    fixed.cylinder("shoulder_seal", 0.137, 0.012, [0, -0.061, 0], BLACK)
    fixed.bolts("shoulder_housing", 0.143, -0.063)
    # The shoulder is passive: only a bearing and encoder, no motor housing.
    upper.cylinder("shoulder_hub", 0.108, 0.105, [0, -0.12, 0], GRAPHITE)
    upper.cylinder("shoulder_face", 0.090, 0.018, [0, -0.182, 0], ALUMINUM)
    upper.cylinder("shoulder_encoder_cap", 0.044, 0.012, [0, -0.199, 0], BLACK)
    upper.bolts("shoulder_hub", 0.067, -0.195, count=4)
    _link(upper, l1, -0.12, ORANGE)

    # The sole actuator is attached to link 1 at the elbow joint.
    upper.cylinder("elbow_motor", 0.145, 0.22, [0, -0.13, -l1], GRAPHITE)
    for index in range(5):
        upper.cylinder(f"motor_cooling_fin_{index}", 0.154, 0.012,
                       [0, -0.038 - index * 0.035, -l1], BLACK)
    upper.cylinder("elbow_gearbox", 0.128, 0.075, [0, -0.277, -l1], ALUMINUM)
    upper.cylinder("elbow_output_seal", 0.102, 0.012, [0, -0.321, -l1], BLACK)
    lower.cylinder("elbow_output_hub", 0.104, 0.10, [0, -0.373, 0], GRAPHITE)
    lower.cylinder("elbow_face", 0.085, 0.012, [0, -0.431, 0], ALUMINUM)
    lower.bolts("elbow_output", 0.063, -0.440, count=6)
    lower.cylinder("elbow_center_cap", 0.032, 0.009, [0, -0.444, 0], ORANGE)
    _link(lower, l2, -0.373, ALUMINUM)
    lower.cylinder("tip_weight", 0.107, 0.14, [0, -0.373, -l2], GRAPHITE)
    lower.cylinder("tip_cover", 0.087, 0.012, [0, -0.451, -l2], ORANGE)
    lower.cylinder("tip_center", 0.036, 0.008, [0, -0.462, -l2], STEEL)
    lower.bolts("tip", 0.062, -0.461, -l2, count=4)

    # Protected cable along the rear edge of link 1, with retaining clips.
    upper.cylinder("motor_cable", 0.009, l1 - 0.26,
                   [0.105, -0.065, -l1 / 2], BLACK, RotationMatrix())
    for index, z in enumerate((-0.22 * l1, -0.5 * l1, -0.78 * l1)):
        upper.box(f"cable_clip_{index}", [0.040, 0.026, 0.018],
                  [0.097, -0.065, z], GRAPHITE)
    fixed.box("power_module", [0.17, 0.16, 0.22], [0.24, 0.38, -0.31], GRAPHITE)
    fixed.cylinder("power_indicator", 0.012, 0.006, [0.24, 0.295, -0.25], BLUE)
    return geometry


def configure_meshcat(meshcat):
    """Frame the upright robot with a clean, softly lit studio background."""
    meshcat.SetCamera(Meshcat.PerspectiveCamera(fov=36, near=0.01, far=100))
    meshcat.SetCameraPose([3.6, -7.0, 3.1], [0, 0.05, 1.20])
    meshcat.SetProperty("/Grid", "visible", False)
    meshcat.SetProperty("/Axes", "visible", False)
    meshcat.SetProperty("/Background", "top_color", [0.94, 0.96, 0.98])
    meshcat.SetProperty("/Background", "bottom_color", [0.79, 0.83, 0.88])
