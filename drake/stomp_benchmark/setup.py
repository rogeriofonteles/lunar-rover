"""Fetch pinned upstream STOMP / UR5 assets and build a ROS-free C ABI bridge."""
from pathlib import Path
import copy
import json
import struct
import subprocess
import urllib.request
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
STOMP = 'b1a87c80f7338caae25a5c689b876da15492aa75'
MODEL = '01645279cbca89c9f9cce00a9b1452bf022a8da1'

def fetch(repo, commit, name, target):
    target.parent.mkdir(parents=True, exist_ok=True)
    if not target.exists():
        url = f'https://raw.githubusercontent.com/{repo}/{commit}/{name}'
        target.write_bytes(urllib.request.urlopen(url, timeout=60).read())

def main():
    vendor = ROOT / 'vendor/stomp'
    for name in ['include/stomp/stomp.h', 'include/stomp/task.h',
                 'include/stomp/utils.h', 'src/stomp.cpp', 'src/utils.cpp', 'LICENSE']:
        fetch('ros-industrial/stomp', STOMP, name, vendor / name)
    assets = ROOT / 'assets'
    prefix = 'robots/ur_description/'
    fetch('Gepetto/example-robot-data', MODEL, prefix + 'urdf/ur5_robot.urdf',
          assets / 'ur5_original.urdf')
    fetch('Gepetto/example-robot-data', MODEL, 'LICENSE', assets / 'LICENSE')
    fetch('Gepetto/example-robot-data', MODEL, prefix + 'README.md', assets / 'UPSTREAM.md')
    tree = ET.parse(assets / 'ur5_original.urdf')
    robot = tree.getroot()
    # Use collision meshes for both roles; convert binary STL to OBJ for Drake.
    for link in robot.findall('link'):
        for visual in link.findall('visual'):
            link.remove(visual)
        for collision in link.findall('collision'):
            mesh = collision.find('geometry/mesh')
            if mesh is None:
                continue
            upstream = mesh.attrib['filename'].removeprefix('package://example-robot-data/')
            stl = assets / Path(upstream).name
            fetch('Gepetto/example-robot-data', MODEL, upstream, stl)
            data = stl.read_bytes()
            n = struct.unpack_from('<I', data, 80)[0]
            assert len(data) == 84 + 50*n, 'Expected binary STL'
            obj = stl.with_suffix('.obj')
            lines = []
            for i in range(n):
                values = struct.unpack_from('<12fH', data, 84 + 50*i)
                for j in (3, 6, 9):
                    lines.append('v ' + ' '.join(str(x) for x in values[j:j+3]))
                lines.append(f'f {3*i+1} {3*i+2} {3*i+3}')
            obj.write_text('\n'.join(lines) + '\n')
            mesh.set('filename', obj.name)
            visual = copy.deepcopy(collision)
            visual.tag = 'visual'
            link.append(visual)
    for tag in ('gazebo', 'transmission'):
        for element in robot.findall(tag):
            robot.remove(element)
    tree.write(assets / 'ur5.urdf')
    (ROOT / 'provenance.json').write_text(json.dumps({
        'stomp_commit': STOMP, 'example_robot_data_commit': MODEL,
        'mesh_processing': 'Binary STL converted to OBJ; Drake uses convex hulls for proximity.'
    }, indent=2) + '\n')
    subprocess.run(['g++', '-O3', '-DNDEBUG', '-std=c++17', '-shared', '-fPIC',
                    '-I/usr/include/eigen3', '-I' + str(vendor / 'include'),
                    str(ROOT / 'bridge.cpp'), str(vendor / 'src/stomp.cpp'),
                    str(vendor / 'src/utils.cpp'), '-lconsole_bridge',
                    '-o', str(ROOT / 'libstomp_bridge.so')], check=True)
    print('Built', ROOT / 'libstomp_bridge.so')

if __name__ == '__main__':
    main()
