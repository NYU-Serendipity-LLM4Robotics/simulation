"""Retrieve the pinned public source files included in this scene catalog."""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import json
import hashlib
import time
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parent
REVISIONS = {
    'robosuite': ('ARISE-Initiative/robosuite', '5ce6643f3092639d08f7b0f90ed1c6a84f50552c'),
    'libero': ('Lifelong-Robot-Learning/LIBERO', '8f1084e3132a39270c3a13ebe37270a43ece2a01'),
    'maniskill': ('mani-skill/ManiSkill', '62ff3a5896b4d5b4cf0ac4c8d79afe600c9404a3'),
    'playground': ('google-deepmind/mujoco_playground', '17ea0152bd7c2cea53d398b82e5c2e8bdd83e25e'),
    'procthor': ('allenai/procthor-10k', '439193522244720b86d8c81cde2e51e3a4d150cf'),
    'khronos': ('KhronosGroup/glTF-Sample-Assets', 'e124e8d8d7330a5372bc2b4d00d39a4a9d7997ea'),
}


def retrieve(url):
    for attempt in range(4):
        try:
            with urlopen(Request(url, headers={'User-Agent': 'Serendipity-scene-library'}), timeout=35) as response:
                return response.read()
        except OSError:
            if attempt == 3:
                raise
            time.sleep(1 + attempt)


def main():
    jobs = []

    def add(family, paths, *, meshes=False, lfs=False):
        repo, rev = REVISIONS[family]
        base = f'https://media.githubusercontent.com/media/{repo}/{rev}' if lfs else f'https://raw.githubusercontent.com/{repo}/{rev}'
        for path in paths:
            destination = ROOT / ('meshes' if meshes else 'native') / family / path
            jobs.append((destination, f'{base}/{path}', family, path, lfs))

    arenas = 'robosuite/models/assets/arenas/'
    add('robosuite', ['LICENSE', 'README.md', *(arenas + name + '_arena.xml' for name in ('table', 'bins', 'pegs', 'multi_table', 'empty')),
                     *('robosuite/models/arenas/' + name + '_arena.py' for name in ('table', 'bins', 'pegs', 'multi_table', 'empty'))])
    add('libero', ['LICENSE', 'README.md', 'libero/libero/benchmark/__init__.py', 'libero/libero/benchmark/libero_suite_task_map.py'])
    repo, rev = REVISIONS['libero']
    tree = json.loads(retrieve(f'https://api.github.com/repos/{repo}/git/trees/{rev}?recursive=1'))
    assert not tree.get('truncated')
    add('libero', [item['path'] for item in tree['tree'] if item['type'] == 'blob' and item['path'].startswith(
        ('libero/libero/bddl_files/', 'libero/libero/init_files/'))])
    add('maniskill', ['LICENSE', 'README.md', 'mani_skill/envs/tasks/humanoid/humanoid_pick_place.py',
                      'mani_skill/envs/tasks/humanoid/transport_box.py', 'mani_skill/envs/tasks/tabletop/pick_cube.py',
                      'mani_skill/envs/tasks/tabletop/stack_cube.py', 'mani_skill/utils/scene_builder/table/scene_builder.py',
                      'mani_skill/envs/tasks/humanoid/assets/apple_1.glb',
                      'mani_skill/envs/tasks/humanoid/assets/frl_apartment_bowl_07.glb',
                      'mani_skill/utils/scene_builder/table/assets/Dining_Table_204_1.glb',
                      'mani_skill/utils/scene_builder/table/assets/table.glb'])
    locomotion = 'rss2025/mujoco_playground/_src/locomotion/'
    add('playground', ['LICENSE', 'rss2025/README.md', *[locomotion + f'{robot}/xmls/{filename}'
                     for robot in ('go1', 'g1') for filename in ('scene_mjx_feetonly_flat_terrain.xml',
                     'scene_mjx_feetonly_rough_terrain.xml', 'assets/hfield.png')],
                     locomotion + 'go1/xmls/scene_mjx_feetonly_stairs.xml'])
    add('procthor', ['LICENSE', 'README.md'])
    add('procthor', [f'{split}.jsonl.gz' for split in ('train', 'val', 'test')], lfs=True)
    for prop in ('SheenChair', 'WaterBottle', 'ToyCar'):
        prefix = f'Models/{prop}/'
        add('khronos', [prefix + 'LICENSE.md', prefix + 'metadata.json', prefix + f'glTF-Binary/{prop}.glb'], meshes=True)

    def fetch(job):
        path, url, family, relative, lfs = job
        if not path.exists():
            content = retrieve(url)
            if lfs:
                repo, rev = REVISIONS[family]
                pointer = retrieve(f'https://raw.githubusercontent.com/{repo}/{rev}/{relative}').decode()
                digest = pointer.split('oid sha256:', 1)[1].splitlines()[0]
                if hashlib.sha256(content).hexdigest() != digest:
                    raise ValueError(f'Git LFS digest mismatch for {relative}')
            path.parent.mkdir(parents=True, exist_ok=True)
            temp = path.with_suffix(path.suffix + '.part')
            temp.write_bytes(content)
            temp.replace(path)
        return {'file': str(path.relative_to(ROOT)), 'source': url, 'bytes': path.stat().st_size,
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}

    with ThreadPoolExecutor(max_workers=12) as pool:
        records = list(pool.map(fetch, jobs))
    (ROOT / 'manifest.json').write_text(json.dumps({'upstream': REVISIONS, 'files': records}, indent=2) + '\n')
    print('Verified catalog', len(records), 'files', sum(item['bytes'] for item in records), 'bytes')


if __name__ == '__main__':
    main()
