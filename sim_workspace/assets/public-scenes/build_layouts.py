"""Convert selected layout geometry to editable Go2/G1 scenes; retain original sources separately."""
import gzip
from io import BytesIO
import json
from math import atan2, degrees, hypot
from pathlib import Path
import re
import sys
from uuid import NAMESPACE_URL, uuid5
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(ROOT.parents[1]))
import mujoco as mj
import numpy as np
import trimesh
from room import StandController
from scene_editor import decode_layout, encode_layout, import_mesh, make_model, new_layout, new_object


def scene(source):
    layout = new_layout(blank=True)
    layout['source_layout'] = source
    layout['poses'] = {'go2/base_link': [-1.25, -.85, .29, 0], 'g1/pelvis': [-.65, .45, .79, 0]}
    return layout


def prop(layout, label, position, size, kind='box', *, fixed=True, yaw=0):
    item = new_object(kind, position, size, label)
    item.update(fixed=fixed, yaw=yaw)
    layout['objects'].append(item)
    return item


def tabletop(layout):
    prop(layout, '工作台', [.65, -.05, .40], [.95, .88, .80])
    return .80


def robosuite(name):
    path = f'robosuite/models/assets/arenas/{name}_arena.xml'
    world = ET.parse(ROOT / 'native' / 'robosuite' / path).getroot()
    layout = scene(path)
    for body in world.findall('./worldbody/body'):
        name = body.get('name', 'fixture')
        if not name.startswith(('table', 'peg', 'bin')):
            continue
        center = np.fromstring(body.get('pos', '0 0 0'), sep=' ') + [.55, 0, 0]
        parts = [g for g in body.findall('./geom') if g.get('type') == 'box' and g.get('group') == '0']
        for i, geom in enumerate(parts):
            xyz = center + np.fromstring(geom.get('pos', '0 0 0'), sep=' ')
            size = 2 * np.fromstring(geom.get('size'), sep=' ')
            prop(layout, f'{name} {i + 1}', xyz.tolist(), size.tolist())
        if name.startswith('bin'):
            prop(layout, f'{name} 支撑', [center[0], center[1], .38], [.34,.44,.76])
    if not layout['objects']:
        raise ValueError(path)
    prop(layout, '抓取块', [.53, -.05, .845], [.04]*3, fixed=False)
    return layout


def stairs():
    path = 'rss2025/mujoco_playground/_src/locomotion/go1/xmls/scene_mjx_feetonly_stairs.xml'
    world = ET.parse(ROOT / 'native' / 'playground' / path).getroot()
    boxes = [g for g in world.findall('.//worldbody//geom') if g.get('type') == 'box']
    layout = scene(path)
    for i, g in enumerate(boxes[:8]):
        x, y, z = map(float, g.get('pos').split())
        dx, dy, dz = map(float, g.get('size').split())
        height = z + dz
        prop(layout, f'台阶 {i+1}', [x - .13, y, height / 2], [2*dx, 2*dy, height])
    return layout


def libero_spatial():
    path = 'libero/libero/bddl_files/libero_spatial/pick_up_the_black_bowl_between_the_plate_and_the_ramekin_and_place_it_on_the_plate.bddl'
    task = (ROOT / 'native' / 'libero' / path).read_text()
    layout = scene(path)
    height = tabletop(layout)
    for zone, label, dimensions in (('plate_region','目标盘', [.13,.13,.015]),
                                    ('between_plate_ramekin_region','目标碗代理', [.08,.08,.045]),
                                    ('ramekin_region','旁边小碗代理', [.08,.08,.038]),
                                    ('box_region','纸盒', [.05,.075,.09])):
        match = re.search(r'\(' + zone + r'\s+\(:target main_table\)\s+\(:ranges\s*\(\s*\(([^)]+)\)', task)
        x0, y0, x1, y1 = map(float, match[1].split())
        prop(layout, label, [.65+(x0+x1)/2, -.05+(y0+y1)/2, height+dimensions[2]/2+.002], dimensions,
             kind='box' if zone == 'box_region' else 'cylinder', fixed=False)
    return layout


def maniskill_stack():
    path = 'mani_skill/envs/tasks/tabletop/stack_cube.py'
    code = (ROOT / 'native' / 'maniskill' / path).read_text()
    half = float(re.search(r'half_size=([0-9.]+)', code)[1])
    layout = scene(path)
    height = tabletop(layout)
    for x, y, label in ((.44,.05,'方块 A'), (.70,-.22,'方块 B')):
        prop(layout, label, [x,y,height+half+.002], [2*half]*3, fixed=False)
    return layout


def procthor(kind):
    with gzip.open(ROOT / 'native' / 'procthor' / 'test.jsonl.gz', 'rt') as data:
        for number, row in enumerate(data):
            house = json.loads(row)
            for room in house['rooms']:
                polygon = room['floorPolygon']
                if room['roomType'] != kind or len(polygon) != 4:
                    continue
                x = [p['x'] for p in polygon]
                y = [p['z'] for p in polygon]
                if not 2.10 <= min(max(x)-min(x), max(y)-min(y)) or max(max(x)-min(x), max(y)-min(y)) > 4.1:
                    continue
                center = [(max(x) + min(x))/2, (max(y) + min(y))/2+.28]
                room_walls = [w for w in house['walls'] if w.get('roomId') == room['id'] and len(w['polygon']) == 4]
                if not room_walls:
                    continue
                layout = scene(f'test.jsonl.gz row={number} room={room["id"]} roomType={kind}')
                layout['poses'] = {'go2/base_link': [-.55, -.66, .29, 0], 'g1/pelvis': [.48, -.18, .79, 0]}
                openings = {d[k] for d in house.get('doors', []) for k in ('wall0', 'wall1') if k in d}
                for wall in room_walls:
                    bottom = [p for p in wall['polygon'] if p['y'] < .02]
                    if len(bottom) != 2:
                        continue
                    ax, ay = bottom[0]['x']-center[0], bottom[0]['z']-center[1]
                    bx, by = bottom[1]['x']-center[0], bottom[1]['z']-center[1]
                    width = hypot(bx-ax, by-ay)
                    height = max(p['y'] for p in wall['polygon'])
                    yaw = degrees(atan2(by-ay, bx-ax))
                    label = f'{kind} wall {len(layout["objects"])+1}'
                    intervals = [(0.,1.)]
                    if wall['id'] in openings and width > 1.12:
                        gap = min(.9/width, .77)
                        intervals = [(0., (1.-gap)/2), ((1.+gap)/2, 1.)]
                    for start, stop in intervals:
                        t = (start+stop)/2
                        prop(layout, label, [ax+(bx-ax)*t, ay+(by-ay)*t, height/2],
                             [.997*width*(stop-start),.045,height], yaw=yaw)
                if len(layout['objects']) >= 3:
                    return layout
    raise ValueError(f'no example for {kind}')


def glb_props():
    layout = scene('Khronos glTF Sample Assets / SheenChair, WaterBottle, ToyCar')
    height = tabletop(layout)
    for name, extent, position, fixed in (('SheenChair', 1., [.06,.79], True),
                                           ('WaterBottle', .16, [.78,.15], False),
                                           ('ToyCar', .24, [.48,-.17], False)):
        path = ROOT / 'meshes' / 'khronos' / 'Models' / name / 'glTF-Binary' / (name + '.glb')
        content = path.read_bytes()
        try:
            item = import_mesh(path.name, content, extent)
        except ValueError:
            mesh = trimesh.load_scene(BytesIO(content), file_type='glb').to_geometry().convex_hull
            item = import_mesh(path.name, mesh.export(file_type='glb'), extent)
            item['label'] += ' collision proxy'
        item.update(position=[*position, item['size'][2]/2 + (.005 if fixed else height+.003)], fixed=fixed)
        layout['objects'].append(item)
    return layout


def verify(name, layout):
    for i, obj in enumerate(layout['objects']):
        obj['id'] = 'asset_' + uuid5(NAMESPACE_URL, f'serendipity:{name}:{i}').hex[:12]
    data = encode_layout(layout)
    model, state, addresses = make_model(decode_layout(data))
    stand = StandController(model, state)
    fixed = [(addresses[o['id']], np.array(o['position'])) for o in layout['objects'] if o['fixed']]
    for _ in range(500):
        stand.apply(state)
        mj.mj_step(model, state)
    assert np.isfinite(state.qpos).all(), name
    for index, xyz in fixed:
        assert np.linalg.norm(state.qpos[index:index+3]-xyz) < .015, name
    (ROOT / 'layouts' / (name + '.json')).write_bytes(data)
    print(f'{name} | {len(layout["objects"])} objects | compiled / 0.5 s physics OK', flush=True)


if __name__ == '__main__':
    for filename, builder in (
        ('robosuite-table', lambda: robosuite('table')), ('robosuite-bins', lambda: robosuite('bins')),
        ('robosuite-pegs', lambda: robosuite('pegs')), ('playground-stairs', stairs),
        ('libero-spatial-bowl-plate', libero_spatial), ('maniskill-stack-cube', maniskill_stack),
        ('procthor-kitchen', lambda: procthor('Kitchen')), ('procthor-living-room', lambda: procthor('LivingRoom')),
        ('khronos-props-table', glb_props),
    ):
        verify(filename, builder())
