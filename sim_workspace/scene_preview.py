"""Low-cost display of the frozen pose in scene editing mode."""
from collections import defaultdict

import mujoco as mj
import numpy as np
import trimesh
from mjviser.scene import merge_geoms


class EditPreview:
    def __init__(self, server, model, data, addresses):
        self.root = server.scene.add_frame('/edit-preview', show_axes=False)
        self.frames = {}
        self.triangles = 0
        self.mesh_count = 0
        names = {model.body(name).id: name for name in addresses}
        by_body = defaultdict(list)
        for i in range(model.ngeom):
            if (model.geom_type[i] != mj.mjtGeom.mjGEOM_PLANE and model.geom_group[i] <= 2
                    and model.geom_rgba[i, 3] > .05 and model.geom(i).name != 'floor_finish'):
                by_body[int(model.geom_bodyid[i])].append(i)
        groups = defaultdict(list)
        for body, geom_ids in by_body.items():
            root = body
            while root and root not in names:
                root = int(model.body_parentid[root])
            display = [i for i in geom_ids if not model.geom_contype[i] and not model.geom_conaffinity[i]]
            if root and '/' in names[root] and display:
                geom_ids = display
            origin = data.xpos[root]
            basis = data.xmat[root].reshape(3, 3)
            rotation = data.xmat[body].reshape(3, 3)
            for geom in geom_ids:
                shape = merge_geoms(model, [geom])
                if not len(shape.faces):
                    continue
                if len(shape.faces) > 900:
                    shape = simplify(shape)
                shape.vertices = (shape.vertices @ rotation.T + data.xpos[body] - origin) @ basis
                color = np.round(model.geom_rgba[geom] * 255).astype(np.uint8)
                shape.visual = trimesh.visual.ColorVisuals(
                    mesh=shape, vertex_colors=np.tile(color, (len(shape.vertices), 1)))
                groups[root].append(shape)
        server.scene.add_grid('/edit-preview/floor', width=10, height=10, cell_size=.25,
                              section_size=1, shadow_opacity=0.)
        for body, parts in groups.items():
            shape = trimesh.util.concatenate(parts)
            self.triangles += len(shape.faces)
            self.mesh_count += 1
            path = f'/edit-preview/entity-{body}'
            if body in names:
                frame = server.scene.add_frame(path, show_axes=False, position=data.xpos[body],
                                               wxyz=data.xquat[body])
                self.frames[names[body]] = frame
                path += '/geometry'
            glb = trimesh.exchange.gltf.export_glb(shape, tree_postprocessor=use_matte_material)
            server.scene.add_glb(path, glb, cast_shadow=False, receive_shadow=False)

    def move(self, name, pose):
        frame = self.frames.get(name)
        if frame is not None:
            frame.position, frame.wxyz = pose[:3], pose[3:7]


def use_matte_material(tree):
    # Keep diffuse colors visible without an environment map.
    tree['materials'] = [{'pbrMetallicRoughness': {'metallicFactor': 0.0, 'roughnessFactor': 1.0}}]
    for mesh in tree['meshes']:
        for primitive in mesh['primitives']:
            primitive['material'] = 0


def simplify(mesh):
    try:
        hull = mesh.convex_hull
        if len(hull.vertices) > 260:
            vertices = hull.vertices[np.linspace(0, len(hull.vertices)-1, 260, dtype=int)]
            hull = trimesh.convex.convex_hull(vertices)
        return hull
    except (ValueError, RuntimeError):
        return mesh.bounding_box
