"""Regenerate the local industrial workcell assets (NumPy + Pillow).

Run from any directory: python arm/models/build_scene.py
The generated XML, OBJ and PNG files are checked in; runtime generation and
network access are not required. Inertias preserve the original six-link arm.
"""
from pathlib import Path
import json
import math
import xml.etree.ElementTree as ET

from PIL import Image, ImageDraw, ImageFont
import numpy as np

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / "assets"
ASSETS.mkdir(exist_ok=True)
TEXTURE_SCALE = 2


def fmt(value):
    if isinstance(value, (tuple, list)):
        return " ".join(f"{x:.12g}" if isinstance(x, (int, float)) else str(x) for x in value)
    return str(value)


def el(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {key: fmt(value) for key, value in attrs.items()})


def save(root, name):
    ET.indent(root, space="  ")
    ET.ElementTree(root).write(ROOT / name, encoding="unicode")
    with (ROOT / name).open("a") as stream:
        stream.write("\n")


def loft(name, rings, corner_fraction=.32):
    """Closed rounded casting with dense, smoothly changing cross sections."""
    vertices, faces = [], []
    sections = []
    for start, end in zip(rings, rings[1:]):
        for t in np.linspace(0, 1, 6, endpoint=False):
            blend = t*t*(3-2*t)
            sections.append((start[0]+t*(end[0]-start[0]),
                             start[1]+blend*(end[1]-start[1]),
                             start[2]+blend*(end[2]-start[2])))
    sections.append(rings[-1])
    corner_points = 9
    count = 4*corner_points
    for z, x, y in sections:
        c = min(x, y) * corner_fraction
        for cx, cy, angle in ((x-c,-y+c,-math.pi/2),(x-c,y-c,0),
                              (-x+c,y-c,math.pi/2),(-x+c,-y+c,math.pi)):
            for theta in np.linspace(angle,angle+math.pi/2,corner_points):
                vertices.append((cx+c*math.cos(theta),cy+c*math.sin(theta),z))
    for k in range(len(sections)-1):
        for j in range(count):
            a, b, c, d = k*count+j, k*count+(j+1)%count, (k+1)*count+(j+1)%count, (k+1)*count+j
            faces.extend([(a, b, c), (a, c, d)])
    for j in range(1, count-1):
        faces.append((0, j+1, j))
        a = (len(sections)-1)*count
        faces.append((a, a+j, a+j+1))
    text = "# Locally generated rounded industrial casting\n"
    text += "".join(f"v {x:.8f} {y:.8f} {z:.8f}\n" for x, y, z in vertices)
    text += "".join("f " + " ".join(str(i+1) for i in face) + "\n" for face in faces)
    (ASSETS / f"{name}.obj").write_text(text)


def label(name, size, background, lines, accent=None):
    scale = TEXTURE_SCALE
    image = Image.new("RGB", (size[0]*scale,size[1]*scale), background)
    draw = ImageDraw.Draw(image)
    if accent:
        draw.rectangle((0, 0, 12*scale, size[1]*scale), fill=accent)
    for text, xy, font_size, color in lines:
        draw.text((xy[0]*scale,xy[1]*scale), text,
                  font=ImageFont.load_default(size=font_size*scale), fill=color)
    image.save(ASSETS / f"{name}.png")


def surface_textures():
    """Fine, low-contrast material grain; deterministic and seamless at edges."""
    rng = np.random.default_rng(6)
    grain = rng.normal(0,.012,(1024,1024))
    # Gentle grain gives the floor a coated-concrete finish without a noisy pattern.
    for name, noise in (("epoxy",grain),("brushed_metal",rng.normal(0,.023,(1024,1))+grain*.15)):
        gray = np.uint8(np.clip(.94+noise,0,1)*255)
        Image.fromarray(np.repeat(gray[:,:,None],3,axis=2)).save(ASSETS/f"{name}.png")


def geom(parent, kind="box", material="steel", **kwargs):
    return el(parent, "geom", type=kind, material=material, **kwargs)


def box(parent, pos, size, material="steel", **kwargs):
    return geom(parent, pos=pos, size=size, material=material, **kwargs)


def cylinder(parent, pos, radius, half, material="steel", axis="z"):
    attrs = {"euler": (math.pi/2, 0, 0)} if axis == "y" else {}
    if axis == "x":
        attrs = {"euler": (0, math.pi/2, 0)}
    return geom(parent, "cylinder", material, pos=pos, size=(radius, half), **attrs)


def line(parent, start, end, radius=.01, material="graphite"):
    return geom(parent, "capsule", material, fromto=(*start, *end), size=(radius,))


def sign(parent, name, pos, width, height):
    # Box texture maps its +Z face; rotate that face to point toward the camera (-Y).
    box(parent, pos, (width/2, height/2, .001), material=name, euler=(math.pi/2, 0, 0))


def bolts(parent, center, radius, count=8, axis="y", size=.007):
    x, y, z = center
    for i in range(count):
        a = i*2*math.pi/count
        p = (x+radius*math.cos(a), y, z+radius*math.sin(a)) if axis == "y" else (x+radius*math.cos(a), y+radius*math.sin(a), z)
        cylinder(parent, p, size, .003, "steel", axis)
        if axis == "y":
            box(parent, (p[0], p[1]-.0032, p[2]), (size*.55, .0008, .0012), "graphite")


def bearing(parent, radius, half=.075, axis="y"):
    cylinder(parent, (0, 0, 0), radius, half, "graphite", axis)
    if axis == "y":
        cylinder(parent, (0, -half-.003, 0), radius*.86, .006, "steel", axis)
        cylinder(parent, (0, -half-.010, 0), radius*.67, .005, "ivory", axis)
        cylinder(parent, (0, -half-.017, 0), radius*.26, .003, "graphite", axis)
        bolts(parent, (0, -half-.018, 0), radius*.73, size=.005)
        cylinder(parent, (0, half+.006, 0), radius*.8, .016, "graphite", axis)
    elif axis == "x":
        cylinder(parent, (.061, 0, 0), radius*.83, .008, "steel", axis)
        cylinder(parent, (.071, 0, 0), radius*.64, .006, "ivory", axis)


def cable(parent, length, offset):
    points = [(0, offset+.012*math.sin(i*math.pi/12), .025+(length-.05)*i/12) for i in range(13)]
    for a, b in zip(points, points[1:]):
        line(parent, a, b, .009, "rubber")
    for z in (.04, length-.04):
        box(parent, (0, offset, z), (.022, .014, .006), "graphite")
        cylinder(parent, (0, offset+.015, z), .006, .003, "steel", "y")


def main():
    loft("upper_casting", [(.025,.058,.047),(.045,.083,.065),(.09,.082,.063),(.26,.062,.05),(.32,.066,.052),(.342,.048,.038)])
    loft("forearm_casting", [(.022,.057,.045),(.04,.07,.055),(.08,.068,.053),(.24,.043,.036),(.28,.047,.038)])
    loft("turret_casting", [(.005,.098,.083),(.025,.11,.09),(.11,.09,.08),(.17,.075,.065)])
    loft("wrist_casting", [(.01,.047,.04),(.025,.054,.043),(.09,.039,.034),(.115,.03,.03)])
    loft("bevel_cube", [(-1,.96,.96),(-.96,1,1),(.96,1,1),(1,.96,.96)],corner_fraction=.08)
    surface_textures()

    label("lab_sign", (1536,384), "#192a35", [
        ("LUNAR / ROBOTICS",(55,45),112,"#f1f4f1"),
        ("MOTION CONTROL LAB    /    CELL 06",(60,230),48,"#a5b7c1")], "#ff8a32")
    label("arm_badge", (512,192), "#202b32", [
        ("AX-06",(27,24),82,"#f6f7f3"),("PRECISION SERIES",(30,127),27,"#b9c4c6")], "#ff842d")
    label("pedestal_badge", (1024,256), "#d6dbda", [
        ("06",(32,20),166,"#273944"),("ROBOT WORKCELL",(278,56),57,"#273944"),
        ("LUNAR INDUSTRIES",(283,148),34,"#667782")], "#f38737")
    label("safety_sign", (640,320), "#edb849", [
        ("CAUTION",(40,25),73,"#202b32"),("AUTOMATED CELL",(42,130),43,"#202b32"),
        ("KEEP CLEAR DURING OPERATION",(43,220),25,"#202b32")])
    label("screen", (768,512), "#11232c", [
        ("LUNAR  /  CONTROL",(36,26),40,"#cce4e5"),
        ("CELL 06",(36,104),76,"#f2f7f4"),
        ("6 AXES    |    TORQUE CONTROL",(40,226),29,"#51cbb9"),
        ("POSITION     VELOCITY     COMMAND",(40,307),26,"#91aab6"),
        ("SIMULATION WORKSTATION",(40,448),26,"#7b99a5")])
    image = Image.open(ASSETS/"screen.png")
    draw = ImageDraw.Draw(image)
    for i, length in enumerate((.7,.95,.45,.82,.6,.9)):
        x = 43+i*115
        draw.rectangle(tuple(v*TEXTURE_SCALE for v in (x,360,x+85,410)),fill="#233f4b")
        draw.rectangle(tuple(v*TEXTURE_SCALE for v in (x,410-int(45*length),x+85,410)),fill="#42b9a7")
    image.save(ASSETS/"screen.png")

    root = ET.Element("mujoco", model="Lunar industrial robotics cell")
    el(root, "compiler", angle="radian", autolimits="true", inertiafromgeom="false", meshdir="assets", texturedir="assets")
    el(root, "option", timestep="0.002", gravity="0 0 -9.81", integrator="RK4")
    el(root, "statistic", center=(0,0,1.3), extent=3.5)
    defaults = el(root,"default")
    el(defaults,"joint",type="hinge",damping=.5,armature=.08,limited="false")
    el(defaults,"geom",contype=0,conaffinity=0,mass=0)
    el(defaults,"motor",ctrllimited="true",ctrlrange=(-80,80))
    visual = el(root,"visual")
    el(visual,"global",offwidth=2560,offheight=1920,azimuth=125,elevation=-13,fovy=46)
    # Four samples is the actual limit of the container's llvmpipe renderer.
    el(visual,"quality",shadowsize=8192,offsamples=4,numslices=80,numstacks=32,numquads=12)
    el(visual,"headlight",diffuse=(.30,.32,.35),ambient=(.26,.28,.31),specular=(.12,.12,.12))
    el(visual,"map",znear=.025,zfar=8,shadowclip=1.05)
    el(visual,"rgba",haze=(.25,.30,.34,1))
    asset = el(root,"asset")
    for name in ("epoxy","brushed_metal"):
        el(asset,"texture",name=name,type="2d",file=f"{name}.png")
    palette = {
        "orange":((.94,.30,.055,1),.36,.5), "ivory":((.77,.81,.81,1),.35,.4),
        "graphite":((.055,.078,.095,1),.35,.45), "steel":((.47,.55,.60,1),.7,.8),
        "rubber":((.023,.031,.035,1),.05,.1), "wall":((.48,.55,.60,1),.12,.2),
        "wall_lower":((.19,.26,.31,1),.18,.3), "floor":((.145,.185,.22,1),.2,.3),
        "seam":((.10,.135,.16,1),.05,.1), "yellow":((.96,.63,.12,1),.2,.2),
        "teal":((.06,.46,.48,1),.3,.5), "red":((.8,.055,.035,1),.4,.4),
        "green":((.08,.78,.39,1),.3,.4), "glass":((.22,.38,.46,1),.6,.7),
        "light":((.85,.94,1,1),.2,.3)}
    for name,(rgba,spec,shine) in palette.items():
        attrs = dict(name=name,rgba=rgba,specular=spec,shininess=shine)
        if name in ("floor","steel"):
            attrs.update(texture="epoxy" if name=="floor" else "brushed_metal",
                         texuniform="true",texrepeat=(2,2))
        if name in ("light","green"):
            attrs["emission"] = .6
        el(asset,"material",**attrs)
    for name in ("upper_casting","forearm_casting","turret_casting","wrist_casting","bevel_cube"):
        el(asset,"mesh",name=name,file=f"{name}.obj",smoothnormal="true")
    for name,scale in (("pedestal_shell",(.53,.43,.26)),("cabinet_shell",(.39,.31,.64))):
        el(asset,"mesh",name=name,file="bevel_cube.obj",scale=scale,smoothnormal="true")
    for name in ("lab_sign","arm_badge","pedestal_badge","safety_sign","screen"):
        el(asset,"texture",name=name,type="2d",file=f"{name}.png")
        el(asset,"material",name=name,texture=name,texrepeat=(1,1),texuniform="false",rgba=(1,1,1,1),specular=.1,emission=.3 if name=="screen" else .05)
    world=el(root,"worldbody")
    el(world,"light",name="key",pos=(1,-2,3.8),dir=(-.25,.5,-1),diffuse=(.60,.58,.54),specular=(.35,.35,.35),directional="true",castshadow="true")
    el(world,"light",name="fill",pos=(-2,-.5,2.8),dir=(.4,.2,-1),diffuse=(.30,.35,.40),directional="true",castshadow="false")
    el(world,"light",name="rim",pos=(0,1.8,3.1),dir=(0,-.6,-1),diffuse=(.28,.33,.35),directional="true",castshadow="false")
    # Camera looks along -Z, with X horizontal and Y vertical.
    el(world,"camera",name="overview",pos=(2.55,-3.4,2.3),xyaxes=(.828,.56,0,-.128,.189,.973),fovy=46)
    el(world,"camera",name="detail",pos=(1.5,-2.1,2.15),xyaxes=(.814,.581,0,-.113,.158,.981),fovy=38)
    camera_pos = np.array([-1.8, -3.0, 2.65])
    camera_z = camera_pos - np.array([0, 0, 1.35])
    camera_z /= np.linalg.norm(camera_z)
    camera_x = np.cross([0, 0, 1], camera_z)
    camera_x /= np.linalg.norm(camera_x)
    camera_y = np.cross(camera_z, camera_x)
    el(world,"camera",name="buttons",pos=tuple(camera_pos),
       xyaxes=tuple(np.r_[camera_x, camera_y]),fovy=40)
    geom(world,"plane","floor",size=(6,6,.1))
    # Floor slab seams and a clearly marked operating area.
    for v in range(-5,6):
        box(world,(v,0,.001),(.006,6,.001),"seam")
        box(world,(0,v,.001),(6,.006,.001),"seam")
    for x in (-1.05,1.05):
        box(world,(x,-.05,.004),(.025,1.05,.002),"yellow")
    for y in (-1.1,1):
        box(world,(0,y,.004),(1.075,.025,.002),"yellow")
    for x in (-.95,.95):
        for i in range(8):
            box(world,(x,-.82+i*.21,.007),(.065,.025,.002),"graphite",euler=(0,0,-.55))

    # Paneled back wall and one side wall; camera-facing sides stay open.
    for x in (-2,-1,0,1,2):
        box(world,(x,2.15,1.72),(.493,.06,1.72),"wall")
        box(world,(x,2.078,.38),(.493,.014,.38),"wall_lower")
    box(world,(0,2.06,.79),(2.5,.022,.025),"steel")
    box(world,(0,2.055,.07),(2.5,.03,.07),"graphite")
    box(world,(-2.5,.45,1.72),(.06,1.7,1.72),"wall_lower")
    for x in (-2.48,2.48):
        box(world,(x,2.02,1.7),(.065,.1,1.7),"graphite")
    # High factory glazing, structural frames, ceiling light strips.
    for x in (-1.7,-.85,0,.85,1.7):
        box(world,(x,2.07,2.88),(.395,.02,.28),"glass")
        for dx in (-.41,.41):
            box(world,(x+dx,2.035,2.88),(.018,.025,.30),"graphite")
        box(world,(x,2.035,2.88),(.40,.025,.015),"steel")
    for x in (-1.6,1.6):
        box(world,(x,.5,3.3),(.10,1.5,.055),"graphite")
        box(world,(x,.5,3.239),(.065,1.34,.008),"light")
    sign(world,"lab_sign",(0,2.035,2.18),2.2,.55)
    sign(world,"safety_sign",(-1.75,2.025,1.43),.53,.265)
    # Conduits and the wall ventilation grille.
    for z in (.96,1.03):
        line(world,(-2.4,2.025,z),(2.25,2.025,z),.012,"steel")
    line(world,(2.25,2.025,1.03),(2.25,2.025,2.5),.012,"steel")
    box(world,(-1.78,2.015,2.15),(.28,.025,.21),"graphite")
    for z in range(9):
        box(world,(-1.78,1.982,1.98+z*.04),(.25,.012,.008),"steel")

    # Heavy machine pedestal, leveling feet, door seams and service panels.
    for x in (-.43,.43):
        for y in (-.34,.34):
            cylinder(world,(x,y,.065),.075,.035,"rubber")
            cylinder(world,(x,y,.13),.023,.04,"steel")
    geom(world,"mesh","graphite",mesh="pedestal_shell",pos=(0,0,.39))
    box(world,(0,-.438,.4),(.49,.013,.235),"ivory")
    box(world,(0,0,.69),(.61,.49,.055),"steel")
    box(world,(0,0,.754),(.59,.47,.009),"graphite")
    sign(world,"pedestal_badge",(0,-.455,.43),.85,.213)
    for x in (-.46,.46):
        for z in (.22,.57):
            cylinder(world,(x,-.456,z),.012,.004,"steel","y")
    for y in (-.30,-.20,-.10,0,.10,.20,.30):
        box(world,(.536,y,.40),(.004,.025,.15),"seam")
    # Robot mounting plate and anchor bolts.
    box(world,(0,0,.777),(.19,.18,.014),"steel")
    for x in (-.15,.15):
        for y in (-.14,.14):
            cylinder(world,(x,y,.797),.014,.008,"graphite")

    # Tool preparation bench to the left, with slotted fixture, vise, and bins.
    for x in (-1.85,-1.13):
        for y in (.55,1.18):
            box(world,(x,y,.41),(.035,.035,.39),"graphite")
            box(world,(x,y,.035),(.065,.065,.025),"rubber")
    box(world,(-1.49,.865,.83),(.49,.43,.045),"ivory")
    box(world,(-1.49,.865,.882),(.48,.42,.008),"steel")
    box(world,(-1.49,.865,.30),(.39,.32,.022),"wall_lower")
    box(world,(-1.50,.67,.905),(.22,.19,.015),"graphite")
    for x in range(5):
        for y in range(4):
            cylinder(world,(-1.67+x*.08,.55+y*.08,.923),.009,.002,"steel")
    box(world,(-1.47,.70,.953),(.13,.07,.025),"steel")
    for x in (-1.57,-1.37):
        box(world,(x,.70,.995),(.025,.075,.035),"graphite")
    cylinder(world,(-1.28,.7,.953),.013,.11,"steel","x")
    for x in (-1.77,-1.45,-1.13):
        box(world,(x,1.13,.935),(.125,.12,.045),"teal")
        box(world,(x,1.13,.983),(.10,.095,.004),"graphite")
        for j in range(3):
            cylinder(world,(x-.06+j*.055,1.13,.997),.018,.012,"steel")

    # Electrical cabinet and HMI on the right, facing into the cell.
    box(world,(1.55,1.22,.16),(.40,.32,.12),"graphite")
    geom(world,"mesh","ivory",mesh="cabinet_shell",pos=(1.55,1.22,.91))
    box(world,(1.55,.898,.91),(.365,.012,.61),"wall_lower")
    box(world,(1.55,.88,1.21),(.29,.025,.215),"graphite")
    sign(world,"screen",(1.55,.851,1.21),.53,.353)
    for x,mat in ((1.35,"green"),(1.48,"graphite"),(1.61,"yellow")):
        cylinder(world,(x,.861,.87),.025,.015,mat,"y")
    cylinder(world,(1.78,.866,.87),.044,.015,"yellow","y")
    cylinder(world,(1.78,.844,.87),.030,.020,"red","y")
    for z in (.48,.54,.60,.66):
        box(world,(1.51,.88,z),(.25,.012,.012),"graphite")
    box(world,(1.85,.872,.56),(.012,.022,.13),"steel")
    cylinder(world,(1.80,1.22,1.67),.018,.12,"steel")
    for z,mat in ((1.78,"red"),(1.84,"yellow"),(1.90,"green")):
        cylinder(world,(1.8,1.22,z),.044,.027,mat)
        cylinder(world,(1.8,1.22,z+.029),.046,.004,"graphite")

    # Reachable teaching targets. The JSON also supplies the simulator's saved
    # joint goals, so rebuilding the scene cannot silently move a target.
    with (ROOT.parent / "button_targets.json").open() as stream:
        buttons = json.load(stream)
    for color, target in buttons.items():
        x, y, z = target["position_m"]
        yaw = math.atan2(y, x)
        pitch = target["tool_pitch_rad"]
        direction = np.array([math.sin(pitch)*math.cos(yaw),
                              math.sin(pitch)*math.sin(yaw), math.cos(pitch)])
        # Separate floor stands spread the targets around the workspace.
        back = np.array([x,y,z]) + .12*direction
        box(world,(back[0],back[1],.035),(.13,.12,.035),"graphite")
        box(world,(back[0],back[1],back[2]/2),(.025,.025,back[2]/2),"steel")
        line(world,tuple(back),tuple(np.array([x,y,z])+.07*direction),.02,"steel")
        # Local +Z points from the button face toward the approaching tool.
        cy, sy = math.cos(yaw/2), math.sin(yaw/2)
        cp, sp = math.cos((pitch-math.pi)/2), math.sin((pitch-math.pi)/2)
        button = el(world,"body",name=f"button_{color}",pos=(x,y,z),
                    quat=(cy*cp,-sy*sp,cy*sp,sy*cp))
        box(button,(0,0,-.054),(.09,.09,.035),"graphite")
        box(button,(0,0,-.017),(.083,.083,.004),"ivory")
        cylinder(button,(0,0,-.012),.057,.006,"steel")
        cylinder(button,(0,0,-.007),.049,.005,"rubber")
        geom(button,"cylinder",color,name=f"button_{color}_cap",
             pos=(0,0,-.006),size=(.043,.006))
        for a in (-.069,.069):
            for b in (-.069,.069):
                cylinder(button,(a,b,-.011),.005,.002,"steel")
        el(button,"site",name=f"button_{color}_target",pos=(0,0,0),
           size=.002,rgba=(0,0,0,0))

    # Explicit original inertias decouple the industrial illustration from physics.
    inertias = [
        (2.7482652533603504,.09,(.014817443880070662,.014817443880070662,.0026434876405759863)),
        (7.412587866145116,.18,(.12264098114583763,.12264098114583763,.010832071668704521)),
        (4.122397880040526,.15,(.04699791233113702,.04699791233113702,.004034796925089665)),
        (1.4752567242880468,.065,(.004631585361532748,.004631585361532748,.001005393445729229)),
        (.8261232572844636,.05,(.001646125481686917,.001646125481686917,.0003976756620336666)),
        (.6088557359104392,.05,(.0010958268292006403,.0010958268292006403,.0002256951367150417))]
    # Fixed pedestal rises by 0.79 m; all relative joint transforms are unchanged.
    cylinder(world,(0,0,.86),.135,.065,"graphite")
    cylinder(world,(0,0,.925),.14,.010,"steel")
    cylinder(world,(0,0,.956),.12,.022,"orange")
    bolts(world,(0,0,.984),.10,axis="z",size=.008)
    cylinder(world,(0,0,.994),.10,.016,"graphite")
    parent = world
    names = ("base_yaw","shoulder","elbow","wrist_roll","wrist_pitch","tool_roll")
    heights = (1.03,.18,.36,.30,.13,.10)
    axes = ((0,0,1),(0,1,0),(0,1,0),(1,0,0),(0,1,0),(0,0,1))
    for i,(name,height,axis,(mass,com,inertia)) in enumerate(zip(names,heights,axes,inertias)):
        body=el(parent,"body",name=name,pos=(0,0,height))
        el(body,"inertial",mass=mass,pos=(0,0,com),diaginertia=inertia)
        el(body,"joint",name=f"j{i+1}",axis=axis)
        if i==0:
            geom(body,"mesh","orange",mesh="turret_casting")
            cylinder(body,(0,0,0),.112,.025,"graphite")
            cylinder(body,(0,0,.03),.113,.006,"steel")
            cable(body,.18,.10)
        elif i in (1,2):
            bearing(body,.095 if i==1 else .079,half=.077 if i==1 else .064)
            geom(body,"mesh","orange",mesh="upper_casting" if i==1 else "forearm_casting")
            length=.36 if i==1 else .30
            y=-.066 if i==1 else -.056
            box(body,(0,y,.18 if i==1 else .15),(.042,.005,.085),"graphite")
            box(body,(0,y-.006,.18 if i==1 else .15),(.034,.002,.073),"ivory")
            for z in (.115,.245) if i==1 else (.09,.21):
                for x in (-.027,.027):
                    cylinder(body,(x,y-.010,z),.004,.002,"steel","y")
            if i==1:
                sign(body,"arm_badge",(0,y-.009,.18),.063,.024)
            cable(body,length,-y+.018)
        elif i==3:
            bearing(body,.058,half=.05,axis="x")
            geom(body,"mesh","ivory",mesh="wrist_casting")
            box(body,(0,-.045,.058),(.025,.006,.035),"graphite")
            cable(body,.13,.055)
        elif i==4:
            bearing(body,.047,half=.045)
            box(body,(0,0,.052),(.032,.03,.043),"orange")
            cylinder(body,(0,0,.094),.039,.008,"steel")
        else:
            cylinder(body,(0,0,.008),.041,.013,"graphite")
            cylinder(body,(0,0,.025),.039,.006,"steel")
            bolts(body,(0,0,.033),.029,count=6,axis="z",size=.004)
            box(body,(0,0,.061),(.049,.025,.024),"graphite")
            box(body,(0,-.027,.061),(.04,.004,.015),"ivory")
            for x in (-.04,.04):
                box(body,(x,0,.110),(.010,.018,.033),"steel")
                box(body,(x*.76,0,.143),(.019,.018,.008),"steel")
                box(body,(x*.55,0,.133),(.005,.018,.012),"rubber")
            el(body,"site",name="tool",pos=(0,0,.11),size=.003,rgba=(.1,.8,.8,.5))
            # A narrow stylus projects beyond the fingers to meet the button face.
            cylinder(body,(0,0,.132),.008,.048,"steel")
            geom(body,"sphere","rubber",pos=(0,0,.18),size=(.01,))
            el(body,"site",name="button_tip",pos=(0,0,.19),size=.002,rgba=(0,0,0,0))
        parent=body
    actuators=el(root,"actuator")
    for i,name in enumerate(("base","shoulder","elbow","wrist_roll","wrist_pitch","tool")):
        el(actuators,"motor",name=name,joint=f"j{i+1}")
    root.insert(0,ET.Comment("Generated by build_scene.py. Visual geometry is contact-free and massless; link inertias are explicit."))
    save(root,"arm6.xml")
    print(f"Generated {ROOT/'arm6.xml'} and local assets")


if __name__=="__main__":
    main()
