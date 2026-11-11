"""Bundle the experiment, its data and MP4s into a single offline HTML file."""
import base64
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parent

def main():
    html=(ROOT/'index.html').read_text()
    html=html.replace('<link rel="stylesheet" href="style.css">','<style>\n'+(ROOT/'style.css').read_text()+'\n</style>')
    html=html.replace('<script src="data.js" defer></script><script src="app.js" defer></script>','')
    data=json.loads((ROOT/'data.json').read_text())
    media={s['config']['id']:'data:video/mp4;base64,'+base64.b64encode((ROOT/'videos'/f"{s['config']['id']}.mp4").read_bytes()).decode() for s in data['scenarios']}
    for name in ['README.md','generate.py','render_videos.py']:
        uri='data:text/plain;charset=utf-8;base64,'+base64.b64encode((ROOT/name).read_bytes()).decode()
        html=html.replace(f'href="{name}"',f'href="{uri}" download="{name}"')
    js=(ROOT/'data.js').read_text()+'\nwindow.ACROBOT_MEDIA='+json.dumps(media)+';\n'+(ROOT/'app.js').read_text()
    html=html.replace('</body>','<script>\n'+js.replace('</script','<\\/script')+'\n</script>\n</body>')
    target=ROOT/'acrobot_interativo.html';target.write_text(html)
    print(f'{target}: {target.stat().st_size/1024/1024:.2f} MiB; {len(media)} embedded videos')

if __name__=='__main__': main()
