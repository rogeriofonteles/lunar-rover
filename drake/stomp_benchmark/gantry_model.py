"""Parameterized surrogate gantry: world X, world Z, yaw, sideways UR5."""
import copy
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from pydrake.all import (RobotDiagramBuilder, SceneGraphCollisionChecker,
    RigidTransform, Box, CoulombFriction, Parallelism)
from benchmark import ROOT, START, TARGETS

DEFAULT = dict(x_limits=[-1.,1.], slide_limits=[0.,.7], second_axis='z', mount_height=.6,
               yaw_limits=[-math.pi,math.pi], translation_scale=.25,
               torso_size=[.18,.28,.40], torso_center=[-.16,0.,-.10])
ARM_NAMES = ['shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
             'wrist_1_joint','wrist_2_joint','wrist_3_joint']

def gantry_names(config):
    return ['gantry_x','gantry_'+config.get('second_axis','z'),'gantry_yaw']

def slide_limits(config):
    return config['slide_limits'] if 'slide_limits' in config else config['z_limits']

def nominal_mount(config):
    midpoint=float(np.mean(slide_limits(config)))
    return np.array([np.mean(config['x_limits']),
        midpoint if config.get('second_axis','z')=='y' else 0.,
        config['mount_height']+(midpoint if config.get('second_axis','z')=='z' else 0.)])
# Fractions of half-travel from the midpoint, plus yaw in degrees.
GANTRY_GOALS = dict(nearby=[.15,.15,10], left_reach=[-.6,.5,-25],
    right_reach=[.6,-.3,25], low_reach=[.4,-.5,15],
    high_fold=[-.4,.7,-20], across_pole=[.5,.4,40])

def model_path(config):
    # Build from the downloaded UR5 model, leaving that original untouched.
    tree=ET.parse(ROOT/'assets/ur5.urdf')
    root=tree.getroot(); root.set('name','gantry_ur5')
    root.remove(root.find("joint[@name='world_joint']"))
    for name in ('x_carriage','z_carriage','yaw_torso'):
        link=ET.SubElement(root,'link',name=name)
        inertial=ET.SubElement(link,'inertial')
        ET.SubElement(inertial,'mass',value='1')
        ET.SubElement(inertial,'inertia',ixx='.01',iyy='.01',izz='.01',
                      ixy='0',ixz='0',iyz='0')
        if name=='yaw_torso':
            collision=ET.SubElement(link,'collision')
            ET.SubElement(collision,'origin',xyz=' '.join(map(str,config['torso_center'])))
            geometry=ET.SubElement(collision,'geometry')
            ET.SubElement(geometry,'box',size=' '.join(map(str,config['torso_size'])))
            visual=copy.deepcopy(collision); visual.tag='visual'
            link.append(visual)
    def joint(name,kind,parent,child,axis,limits,xyz='0 0 0',rpy='0 0 0'):
        node=ET.SubElement(root,'joint',name=name,type=kind)
        ET.SubElement(node,'parent',link=parent); ET.SubElement(node,'child',link=child)
        ET.SubElement(node,'origin',xyz=xyz,rpy=rpy)
        if axis:
            ET.SubElement(node,'axis',xyz=axis)
            ET.SubElement(node,'limit',lower=str(limits[0]),upper=str(limits[1]),
                          effort='1000',velocity='1')
    joint('gantry_x','prismatic','world','x_carriage','1 0 0',config['x_limits'],
          xyz=f"0 0 {config['mount_height']}")
    axis=config.get('second_axis','z')
    joint('gantry_'+axis,'prismatic','x_carriage','z_carriage',
          '0 0 1' if axis=='z' else '0 1 0',slide_limits(config))
    joint('gantry_yaw','revolute','z_carriage','yaw_torso','0 0 1',config['yaw_limits'])
    # R_y(+90 deg) maps the UR5 base's +Z joint axis to torso +X.
    joint('torso_arm_mount','fixed','yaw_torso','base_link',None,None,
          rpy=f'0 {math.pi/2} 0')
    # Unique filenames allow independent configurations without overwriting assets.
    import hashlib
    digest=hashlib.sha256(json.dumps(config,sort_keys=True).encode()).hexdigest()[:12]
    path=ROOT/'assets'/f'gantry_{digest}.urdf'
    tree.write(path)
    return path

def indices_and_scales(plant,config):
    indices={name:plant.GetJointByName(name).position_start()
             for name in ARM_NAMES+gantry_names(config)}
    scales=np.ones(plant.num_positions())
    for name in gantry_names(config)[:2]: scales[indices[name]]=config['translation_scale']
    return indices,scales

def start_and_goal(plant,config,target,moving=False):
    indices,scales=indices_and_scales(plant,config)
    start=np.zeros(9)
    for name,q in zip(ARM_NAMES,START): start[indices[name]]=q
    names=gantry_names(config)
    for name,limits in zip(names[:2],(config['x_limits'],slide_limits(config))):
        start[indices[name]]=np.mean(limits)
    goal=start.copy()
    for name,q in zip(ARM_NAMES,np.deg2rad(TARGETS[target])): goal[indices[name]]=q
    if moving:
        fractions=GANTRY_GOALS[target]
        for name,limits,fraction in zip(names[:2],(config['x_limits'],slide_limits(config)),fractions[:2]):
            goal[indices[name]] += fraction*np.diff(limits)[0]/2
        goal[indices['gantry_yaw']]=np.deg2rad(fractions[2])
    return start,goal

def obstacle(config):
    # Rotate/translate the previous pole with the nominal arm mounting frame.
    # This preserves the arm-relative obstruction for the locked/free comparison.
    return RigidTransform(nominal_mount(config)+[.4,.50,0.]), Box(.80,.12,.12)

def make_gantry_scene(scene,config):
    builder=RobotDiagramBuilder(time_step=0.)
    robot=builder.parser().AddModels(str(model_path(config)))[0]
    plant=builder.plant()
    if scene=='bar':
        transform,shape=obstacle(config)
        plant.RegisterCollisionGeometry(plant.world_body(),transform,shape,
                                         'crossbar',CoulombFriction(.9,.5))
    plant.Finalize()
    assert plant.num_positions()==9
    indices,scales=indices_and_scales(plant,config)
    checker=SceneGraphCollisionChecker(model=builder.Build(),robot_model_instances=[robot],
        edge_step_size=.025,env_collision_padding=0.,self_collision_padding=0.,
        implicit_context_parallelism=Parallelism(1),
        configuration_distance_function=lambda a,b:float(np.linalg.norm((a-b)/scales)))
    diagram=checker.model(); context=diagram.CreateDefaultContext()
    plant=diagram.plant(); pc=plant.GetMyMutableContextFromRoot(context)
    sg=diagram.scene_graph(); sc=sg.GetMyMutableContextFromRoot(context)
    return checker,plant,pc,sg.get_query_output_port(),sc
