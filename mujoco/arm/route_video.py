"""Render recorded nonlinear states in the actual industrial MuJoCo scene."""
from pathlib import Path
import subprocess

import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont

from .planning import show_planning_obstacles

COLORS={"red":"vermelho","yellow":"amarelo","green":"verde"}


def render_route_video(arm,arrays,summary,path,width=1920,height=1080,fps=30):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    show_planning_obstacles(arm.model)
    arm.model.vis.global_.offwidth=width;arm.model.vis.global_.offheight=height
    data=mujoco.MjData(arm.model)
    times=arrays['times'];states=arrays['states']
    frame_times=np.r_[np.zeros(fps),np.arange(0,times[-1],1/fps),np.full(2*fps,times[-1])]
    # The Pillow default font omits Portuguese accents and arrow glyphs.
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    font=ImageFont.truetype(font_path,size=round(height*.029))
    small=ImageFont.truetype(font_path,size=round(height*.021))
    command=['ffmpeg','-hide_banner','-loglevel','error','-y','-f','rawvideo','-pix_fmt','rgb24',
             '-s',f'{width}x{height}','-r',str(fps),'-i','pipe:0','-an','-c:v','libx264',
             '-threads','2','-crf','18','-preset','medium','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    with subprocess.Popen(command,stdin=subprocess.PIPE) as encoder:
        try:
            with mujoco.Renderer(arm.model,height=height,width=width) as renderer:
                for index,t in enumerate(frame_times):
                    data.qpos[:]=[np.interp(t,times,states[:,j]) for j in range(6)]
                    data.qvel[:]=0
                    mujoco.mj_forward(arm.model,data)
                    renderer.update_scene(data,camera='buttons')
                    # Dashed reference trail is visual only; arm poses come from MuJoCo integration.
                    for point in arrays['planned_tip'][::8]:
                        scene=renderer.scene
                        if scene.ngeom>=scene.maxgeom:
                            break
                        mujoco.mjv_initGeom(scene.geoms[scene.ngeom],mujoco.mjtGeom.mjGEOM_SPHERE,
                            np.array([.0035,0,0]),point,np.eye(3).ravel(),np.array([.25,.7,1.,.7],dtype=np.float32))
                        scene.ngeom+=1
                    image=Image.fromarray(renderer.render());draw=ImageDraw.Draw(image)
                    x,y=40,30
                    draw.rounded_rectangle((20,15,width*.62,155),radius=12,fill='#132a26')
                    draw.text((x,y),'RRT-CONNECT  /  '+summary['label'],font=font,fill='#f5f4ed')
                    draw.text((x,y+43),' → '.join(COLORS[c] for c in summary['sequence']),font=small,fill='#e6b998')
                    draw.text((x,y+78),f"MuJoCo · referência em azul · tempo simulado {t:.2f} s".replace('.',','),font=small,fill='#d5e3de')
                    encoder.stdin.write(image.tobytes())
                    if index==0:
                        image.save(path.with_name(path.stem+'-start.png'))
                    if index==len(frame_times)-1:
                        image.save(path.with_suffix('.png'))
                    if index%(fps*5)==0:
                        print(f"Vídeo {summary['key']}: {index}/{len(frame_times)} quadros",flush=True)
        finally:
            encoder.stdin.close();code=encoder.wait()
        if code:
            raise RuntimeError(f'FFmpeg terminou com código {code}')
    return dict(file=path.name,width=width,height=height,fps=fps,frames=len(frame_times),
                initial_hold_seconds=1,final_hold_seconds=2,duration_seconds=len(frame_times)/fps)
