"""Browser layout editor using the same MuJoCo models as the tabletop trials."""
import base64
import logging
from copy import deepcopy
from dataclasses import dataclass
from io import BytesIO
import json
import math
from pathlib import Path
import re
from threading import RLock
from time import monotonic, sleep
from uuid import uuid4

import mujoco as mj
import numpy as np
import trimesh
import viser
from mjviser.scene import ViserMujocoScene

from room import StandController
from scene_preview import EditPreview


SCENE_DIR = Path(__file__).resolve().parent / 'runs' / 'scenes'
LIBRARY_DIR = Path(__file__).resolve().parent / 'assets' / 'public-scenes' / 'layouts'
MAX_UPLOAD = 12 * 1024 * 1024
LOG = logging.getLogger('scene_editor')
FORMAT = 'serendipity-scene-v1'
DEFAULT_ASSET_RGB = (59, 122, 171)
LABELS = {'go2/base_link': 'Go2', 'g1/pelvis': 'G1', 'source_table': '工作台',
          'apple': '红球', 'block': '蓝方块', 'cylinder': '紫色圆柱',
          'green_tray': '绿托盘', 'yellow_tray': '黄托盘'}
PRIMITIVES = {'box': ('方块', (.24, .24, .24)), 'sphere': ('球', (.2, .2, .2)),
              'cylinder': ('圆柱', (.22, .22, .28)), 'platform': ('平台', (.65, .38, .12))}


def new_layout(blank=False):
    return {'format': FORMAT, 'blank': blank, 'poses': {}, 'objects': []}


def asset_rgb(item):
    return tuple(item.get('rgb', DEFAULT_ASSET_RGB))


def hex_rgb(rgb):
    return '#' + ''.join(f'{channel:02X}' for channel in rgb)


def new_object(kind, position, size=None, label=None, mesh=None):
    if size is None:
        label, size = PRIMITIVES[kind]
    return {'id': 'asset_' + uuid4().hex[:12], 'kind': kind, 'label': label, 'rgb': list(DEFAULT_ASSET_RGB),
            'size': list(size), 'position': list(position), 'mass': 1., 'yaw': 0.,
            **({'stl': base64.b64encode(mesh).decode('ascii')} if mesh else {})}


def scene_asset():
    layout = new_layout(blank=True)
    layout['objects'] = [new_object('platform', [x, y, .12]) for x, y in
                         [(0., -.7), (.65, .0), (1.30, -.7)]]
    layout['objects'].append(new_object('cylinder', [.65, -.68, .28]))
    layout['poses'] = {'go2/base_link': [-1.65, -.55, .29, 0.],
                       'g1/pelvis': [-1.37, .62, .79, 0.]}
    return layout


def numeric_vector(value, length, limit):
    if not isinstance(value, list) or len(value) != length or not all(
            type(x) in (float, int) and math.isfinite(x) and abs(x) <= limit for x in value):
        raise ValueError('坐标、角度或尺寸超出范围')
    return value


def decode_layout(content):
    if len(content) > MAX_UPLOAD:
        raise ValueError('场景文件超过 12 MB')
    layout = json.loads(content)
    if (not isinstance(layout, dict) or layout.get('format') != FORMAT or
            type(layout.get('blank')) is not bool):
        raise ValueError('请选择本编辑器导出的场景 JSON')
    poses, objects = layout.get('poses'), layout.get('objects')
    if not isinstance(poses, dict) or not isinstance(objects, list) or len(objects) > 30:
        raise ValueError('场景结构无效，每个场景最多添加 30 个物体')
    for name, pose in poses.items():
        if name not in LABELS:
            raise ValueError('未知内置物体')
        numeric_vector(pose, 4, 360)
        numeric_vector(pose[:3], 3, 30)
    ids = set()
    for item in objects:
        if not isinstance(item, dict) or not re.fullmatch(r'asset_[0-9a-f]{12}', str(item.get('id'))):
            raise ValueError('物体 ID 无效')
        if item['id'] in ids:
            raise ValueError('场景有重复物体')
        ids.add(item['id'])
        kind = item.get('kind')
        if not isinstance(kind, str) or kind not in (*PRIMITIVES, 'mesh'):
            raise ValueError('未知物体类型')
        if not isinstance(item.get('label'), str) or not 0 < len(item['label']) <= 80:
            raise ValueError('物体名称无效')
        rgb = item.get('rgb', list(DEFAULT_ASSET_RGB))
        if (not isinstance(rgb, list) or len(rgb) != 3
                or any(type(channel) is not int or not 0 <= channel <= 255 for channel in rgb)):
            raise ValueError('物体颜色 RGB 必须是三个 0–255 的整数')
        item['rgb'] = rgb
        numeric_vector(item.get('position'), 3, 30)
        if min(numeric_vector(item.get('size'), 3, 8)) < .01:
            raise ValueError('物体尺寸必须至少为 1 cm')
        mass, yaw = numeric_vector([item.get('mass'), item.get('yaw')], 2, 360)
        if not .01 <= mass <= 100 or abs(yaw) > 180:
            raise ValueError('质量或角度超出范围')
        if 'fixed' in item and type(item['fixed']) is not bool:
            raise ValueError('固定状态必须为布尔值')
        if kind == 'mesh':
            stl = base64.b64decode(item['stl'], validate=True)
            if len(stl) < 84 or len(stl) > 2_000_084:
                raise ValueError('网格大小无效')
    return layout


def encode_layout(layout):
    return (json.dumps(layout, ensure_ascii=False, indent=2, allow_nan=False) + '\n').encode()


def import_mesh(filename, content, target_size):
    source = Path(filename)
    ext = source.suffix.lower()[1:]
    if ext not in ('obj', 'stl', 'glb') or len(content) > MAX_UPLOAD:
        raise ValueError('请选择不超过 12 MB 的 OBJ、STL 或 GLB')
    mesh = trimesh.load_scene(BytesIO(content), file_type=ext).to_geometry()
    if not isinstance(mesh, trimesh.Trimesh) or not 4 <= len(mesh.faces) <= 40000:
        raise ValueError('请使用含 4–40,000 个三角面的三维网格')
    if ext == 'glb':
        mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi/2, [1, 0, 0]))
    mesh.apply_translation(-mesh.bounds.mean(axis=0))
    dimension = float(mesh.extents.max())
    if not np.isfinite(dimension) or dimension <= 0 or min(mesh.extents) / dimension < 1e-5:
        raise ValueError('网格需要有厚度和有限的顶点坐标')
    mesh.apply_scale(target_size / dimension)
    label = source.stem[:70] or '导入网格'
    return new_object('mesh', [.1, .25, float(mesh.extents[2]/2 + .05)],
                      np.maximum(mesh.extents, .01), label, mesh.export(file_type='stl'))


def make_model(layout):
    from workspace import scene

    world, meshes, welds, files = [], [], [], {}
    for item in layout['objects']:
        key, kind, size = item['id'], item['kind'], np.asarray(item['size'])
        rgb = ' '.join(f'{channel / 255:.8f}' for channel in asset_rgb(item))
        if kind == 'mesh':
            filename = key + '.stl'
            files[filename] = base64.b64decode(item['stl'], validate=True)
            meshes.append(f'<mesh name="{key}" file="{filename}" inertia="convex"/>')
            shape = f'type="mesh" mesh="{key}"'
        else:
            shape_type = 'box' if kind == 'platform' else kind
            half = size / 2
            dims = {'box': half, 'sphere': half[:1], 'cylinder': half[[0, 2]]}[shape_type]
            shape = f'type="{shape_type}" size="' + ' '.join(map(str, dims)) + '"'
        if item.get('fixed', False):
            pose = ' '.join(map(str, [*item['position'], *yaw_quaternion(item['yaw'])]))
            welds.append(f'<weld name="fix_{key}" body1="world" body2="{key}" relpose="{pose}" solref=".002 1"/>')
        world.append(f'<body name="{key}"><freejoint/><geom name="{key}" {shape} '
                     f'mass="{item["mass"]}" rgba="{rgb} 1" friction=".65 .008 .001"/></body>')
    model, data, _ = scene(0, physics_room=True, lab_layout=True, empty_lab=layout['blank'],
                           extra_world=''.join(world), extra_meshes=''.join(meshes), mesh_files=files,
                           extra_equalities='<equality>' + ''.join(welds) + '</equality>' if welds else '')
    names = set(LABELS) | {item['id'] for item in layout['objects']}
    addresses = {model.body(model.jnt_bodyid[j]).name: int(model.jnt_qposadr[j])
                 for j, kind in enumerate(model.jnt_type) if kind == mj.mjtJoint.mjJNT_FREE
                 and model.body(model.jnt_bodyid[j]).name in names}
    poses = {**layout['poses'], **{item['id']: [*item['position'], item['yaw']] for item in layout['objects']}}
    for name, pose in poses.items():
        if name in addresses:
            set_pose(data, addresses[name], pose[:3], pose[3])
    mj.mj_forward(model, data)
    return model, data, addresses


def yaw_quaternion(yaw):
    half_angle = math.radians(yaw) / 2
    return math.cos(half_angle), 0., 0., math.sin(half_angle)


def set_pose(data, index, xyz, yaw):
    data.qpos[index:index + 7] = [*xyz, *yaw_quaternion(yaw)]


@dataclass
class Selection:
    name: str
    address: int
    obj: dict | None


class Editor:
    def __init__(self, server, layout):
        self.server = server
        self.lock = RLock()
        self.playing = False
        self.gizmo = None
        self.pending_drag = None
        self.next_drag_frame = 0.
        self.markers = {}
        gui = server.gui
        gui.add_markdown('## MuJoCo 场景编辑器')
        self.status = gui.add_markdown('编辑模式 · 拖动箭头摆放物体')
        with gui.add_folder('场景与仿真'):
            self.play = gui.add_button('播放物理', color='teal')
            self.stop = gui.add_button('返回布置')
            self.template = gui.add_dropdown('模板', ['普通桌面', '空房间', '障碍练习场'])
            self.load_template = gui.add_button('载入模板')
        with gui.add_folder('选中物体'):
            self.pick = gui.add_dropdown('物体', ['Go2'])
            self.position = gui.add_vector3('XYZ 米', (0., 0., 0.), min=(-30.,)*3, max=(30.,)*3, step=.005)
            self.yaw = gui.add_number('朝向 °', 0., min=-180., max=180., step=1.)
            self.color = gui.add_rgb('物体颜色 RGB', DEFAULT_ASSET_RGB, visible=False)
            self.apply_color = gui.add_button('应用颜色', visible=False)
            self.material = gui.add_markdown('')
            self.mass = gui.add_number('质量 kg', 1., min=.01, max=100., step=.05)
            self.fixed = gui.add_checkbox('固定于场景', initial_value=False)
            self.remove = gui.add_button('移除选中添加物')
        with gui.add_folder('添加资产'):
            self.kind = gui.add_dropdown('形状', [x[0] for x in PRIMITIVES.values()])
            self.size = gui.add_number('最长边 m', .24, min=.02, max=5., step=.01)
            self.add = gui.add_button('添加形状', color='blue')
            self.mesh = gui.add_upload_button('导入 OBJ / STL / GLB')
        with gui.add_folder('保存与导入'):
            self.filename = gui.add_text('场景名称', initial_value='my-room')
            self.save = gui.add_button('保存到服务器')
            self.saved = gui.add_dropdown('服务器场景', ['请选择'])
            self.open_saved = gui.add_button('打开服务器场景')
            self.download = gui.add_button('下载场景 JSON')
            self.upload = gui.add_upload_button('导入场景 JSON')
            self.example = gui.add_button('下载障碍练习场资产')
        self.load(layout)
        self.refresh_saved()
        self.bind_events()

    def action(self, name, callback, *, edits=True, log_success=True):
        def handle(event):
            if event.client is None:
                return
            with self.lock:
                try:
                    if edits and self.playing:
                        raise ValueError('先点击「返回布置」')
                    callback(event)
                except Exception as exc:
                    ref = uuid4().hex[:12]
                    if isinstance(exc, (ValueError, FileExistsError)):
                        LOG.warning('action_failed action=%s ref=%s', name, ref, exc_info=True)
                    else:
                        LOG.exception('action_failed action=%s ref=%s', name, ref)
                    message = next(iter(str(exc).splitlines()), type(exc).__name__)
                    self.status.content = f'操作未完成 · {message[:180]} · 日志编号 {ref}'
                else:
                    if log_success:
                        LOG.info('action_ok action=%s', name)
        return handle

    def bind_events(self):
        self.pick.on_update(self.action('select_object', lambda _: self.select()))
        self.position.on_update(self.action('set_position', lambda _: self.move(self.position.value, self.yaw.value)))
        self.yaw.on_update(self.action('set_yaw', lambda _: self.move(self.position.value, self.yaw.value)))
        self.apply_color.on_click(self.action('apply_color', lambda _: self.set_asset_property('rgb', list(self.color.value))))
        self.mass.on_update(self.action('set_mass', lambda _: self.set_asset_property('mass', float(self.mass.value))))
        self.fixed.on_update(self.action('set_fixed', lambda _: self.set_asset_property('fixed', self.fixed.value)))
        self.add.on_click(self.action('add_shape', lambda _: self.add_primitive()))
        self.remove.on_click(self.action('remove_object', lambda _: self.remove_asset()))
        self.mesh.on_upload(self.action('import_mesh', lambda event: self.add_asset(import_mesh(
            event.target.value.name, event.target.value.content, self.size.value))))
        self.upload.on_upload(self.action('import_scene', lambda event: self.load(decode_layout(event.target.value.content))))
        self.save.on_click(self.action('save_scene', lambda _: self.save_layout()))
        self.open_saved.on_click(self.action('open_saved_scene', lambda _: self.load_saved()))
        self.download.on_click(self.action('download_scene', lambda event: event.client.send_file_download(
            self.file_name(), encode_layout(self.layout))))
        self.example.on_click(self.action('download_example', lambda event: event.client.send_file_download(
            'obstacle-course.json', encode_layout(scene_asset())), edits=False))
        self.load_template.on_click(self.action('load_template', lambda _: self.load(
            scene_asset() if self.template.value == '障碍练习场' else new_layout(self.template.value == '空房间'))))
        self.play.on_click(self.action('play_physics', lambda _: self.start()))
        self.stop.on_click(self.action('restore_layout', lambda _: self.restore(), edits=False))

    def load(self, layout, select_name=None):
        layout = decode_layout(encode_layout(layout))
        model, data, addresses = make_model(layout)
        self.layout, self.model, self.data, self.addresses = layout, model, data, addresses
        self.server.scene.reset()
        self.server.scene.add_light_ambient('/editor-ambient', intensity=0.95)
        self.preview = EditPreview(self.server, model, data, addresses)
        self.pending_drag = None
        self.visual = None
        assets = {item['id']: item for item in layout['objects']}
        self.entities = {}
        self.markers = {}
        for name in [*LABELS, *assets]:
            if name not in addresses:
                continue
            item = assets.get(name)
            label = LABELS.get(name)
            if item is not None:
                label = f'{item["label"]} · {hex_rgb(asset_rgb(item))} · {name[-4:]}'
            self.entities[label] = Selection(name, addresses[name], item)
            marker = self.server.scene.add_icosphere('/editor/pick/' + name.replace('/', '_'), radius=.025,
                                                    color=(38, 144, 196), subdivisions=1, opacity=.85,
                                                    cast_shadow=False, receive_shadow=False)
            marker.on_click(self.action('select_object', lambda _, label=label: self.select(label)))
            self.markers[label] = marker
        self.gizmo = None
        self.pick.options = tuple(self.entities)
        self.select(next((label for label, item in self.entities.items() if item.name == select_name), 'Go2'))
        self.refresh_markers()
        self.status.content = '编辑模式 · 点击蓝点或选择物体，再拖动坐标轴'

    def select(self, label=None):
        self.pending_drag = None
        if label is not None:
            self.pick.value = label
        selected = self.entities[self.pick.value]
        state = self.data.qpos[selected.address:selected.address + 7]
        self.position.value = tuple(state[:3])
        self.yaw.value = math.degrees(2 * math.atan2(state[6], state[3]))
        self.color.visible = self.apply_color.visible = selected.obj is not None
        self.material.content = ''
        if selected.obj is not None:
            rgb = asset_rgb(selected.obj)
            self.color.value = rgb
            self.material.content = f'已应用颜色 {hex_rgb(rgb)} · ID {selected.name}'
        self.mass.disabled = selected.obj is None
        self.fixed.disabled = selected.obj is None
        self.fixed.value = selected.obj is not None and selected.obj.get('fixed', False)
        self.remove.disabled = selected.obj is None
        if selected.obj is not None:
            self.mass.value = selected.obj['mass']
        if self.gizmo is not None:
            self.gizmo.remove()
        self.gizmo = self.server.scene.add_transform_controls('/editor/gizmo', position=state[:3],
                                                              scale=.38, disable_rotations=True, depth_test=False,
                                                              translation_limits=((-30., 30.),)*3)
        self.gizmo.on_update(self.action('queue_drag', self.queue_drag, log_success=False))
        self.gizmo.on_drag_end(self.action('finish_drag', self.finish_drag))

    def queue_drag(self, event):
        if event.target is self.gizmo:
            self.pending_drag = tuple(event.target.position)

    def finish_drag(self, event):
        if event.target is self.gizmo:
            self.pending_drag = None
            self.move(event.target.position, self.yaw.value, drag=True)

    def refresh_markers(self, only=None):
        labels = (only,) if only is not None else self.markers
        for label in labels:
            marker = self.markers[label]
            item = self.entities[label]
            offset = .10 if item.obj is None else item.obj['size'][2] / 2 + .08
            if item.name == 'source_table':
                offset = .93
            marker.position = self.data.qpos[item.address:item.address+3] + [0., 0., offset]

    def move(self, xyz, yaw, *, drag=False, update_form=True):
        selected = self.entities[self.pick.value]
        position = numeric_vector(list(map(float, xyz)), 3, 30)
        yaw = float(yaw)
        if not math.isfinite(yaw) or abs(yaw) > 180.:
            raise ValueError('朝向必须在 ±180° 内')
        set_pose(self.data, selected.address, position, yaw)
        if selected.obj is None:
            self.layout['poses'][selected.name] = [*position, yaw]
        else:
            selected.obj.update(position=position, yaw=yaw)
            if selected.obj.get('fixed', False):
                equality = self.model.equality('fix_' + selected.name).id
                self.model.eq_data[equality, 3:10] = self.data.qpos[selected.address:selected.address+7]
        self.preview.move(selected.name, self.data.qpos[selected.address:selected.address+7])
        if drag and update_form:
            self.position.value = tuple(position)
        elif not drag:
            self.gizmo.position = position
        self.refresh_markers(self.pick.value)

    def add_primitive(self):
        kind = next(key for key, value in PRIMITIVES.items() if value[0] == self.kind.value)
        label, size = PRIMITIVES[kind]
        size = np.asarray(size) * (self.size.value / max(size))
        self.add_asset(new_object(kind, [.1, .25, float(size[2]/2 + .05)], size, label))

    def add_asset(self, item):
        layout = deepcopy(self.layout)
        layout['objects'].append(item)
        self.load(layout, item['id'])
        self.status.content = '已添加 · ' + item['label']

    def remove_asset(self):
        selection = self.entities[self.pick.value]
        if selection.obj is None:
            raise ValueError('请先选择自行添加的物体')
        layout = deepcopy(self.layout)
        layout['objects'] = [item for item in layout['objects'] if item['id'] != selection.name]
        self.load(layout)

    def set_asset_property(self, field, value):
        selection = self.entities[self.pick.value]
        if selection.obj is None:
            return
        layout = deepcopy(self.layout)
        for item in layout['objects']:
            if item['id'] == selection.name:
                item[field] = value
        self.load(layout, selection.name)

    def file_name(self):
        name = self.filename.value.strip()
        if not re.fullmatch(r'[\w-]{1,48}', name):
            raise ValueError('场景名只用字母、数字、中文、下划线或短横线')
        return name + '.json'

    def refresh_saved(self):
        self.saved_paths = {p.name: p for p in sorted(SCENE_DIR.glob('*.json'))}
        self.saved_paths.update({'公开 / ' + p.stem: p for p in sorted(LIBRARY_DIR.glob('*.json'))})
        self.saved.options = ['请选择', *self.saved_paths]

    def save_layout(self):
        SCENE_DIR.mkdir(parents=True, exist_ok=True)
        path = SCENE_DIR / self.file_name()
        with path.open('xb') as file:
            file.write(encode_layout(self.layout))
        self.refresh_saved()
        self.saved.value = path.name
        self.status.content = '已保存 · ' + path.name

    def load_saved(self):
        name = self.saved.value
        if name == '请选择':
            raise ValueError('请从列表选择一个场景')
        self.load(decode_layout(self.saved_paths[name].read_bytes()))
        self.status.content = '已打开 · ' + name

    def start(self):
        self.pending_drag = None
        mj.mj_forward(self.model, self.data)
        self.visual = ViserMujocoScene(self.server, self.model, num_envs=1)
        self.visual.camera_tracking_enabled = False
        self.visual.update_from_mjdata(self.data)
        self.preview.root.visible = False
        self.stand = StandController(self.model, self.data)
        self.gizmo.visible = False
        for marker in self.markers.values():
            marker.visible = False
        self.playing = True
        self.play.disabled = True
        self.status.content = '实时物理 · 重力与碰撞已启用'

    def restore(self):
        self.playing = False
        self.play.disabled = False
        self.load(self.layout)

    def tick(self):
        with self.lock:
            if not self.playing:
                now = monotonic()
                if self.pending_drag is not None and now >= self.next_drag_frame:
                    position, self.pending_drag = self.pending_drag, None
                    self.next_drag_frame = now + 1/30
                    self.move(position, self.yaw.value, drag=True, update_form=False)
                return
            for _ in range(round(.02 / self.model.opt.timestep)):
                self.stand.apply(self.data)
                mj.mj_step(self.model, self.data)
            self.visual.update_from_mjdata(self.data)


def run(port, path):
    logging.basicConfig(level=logging.INFO, format='%(asctime)s %(levelname)s %(name)s %(message)s')
    layout = decode_layout(path.read_bytes()) if path else new_layout()
    server = viser.ViserServer(host='127.0.0.1', port=port, label='Serendipity Scene Editor')
    server.initial_camera.position = (-1.9, -3.2, 2.45)
    server.initial_camera.look_at = (.15, -.48, .48)
    server.initial_camera.up = (0., 0., 1.)
    server.scene.configure_default_lights(cast_shadow=False)
    server.scene.configure_environment_map(hdri=None)
    try:
        editor = Editor(server, layout)
        LOG.info('editor_ready port=%d layout=%s', port, path or 'default')
        while True:
            start = monotonic()
            editor.tick()
            sleep(max(.001, .02 - (monotonic() - start)))
    finally:
        server.stop()
