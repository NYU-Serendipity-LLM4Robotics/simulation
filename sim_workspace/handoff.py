"""Versioned scout-to-manipulator message for the shared simulated world."""
from math import isfinite

VERSION = 1
FRAME = 'world'
POSITION_KINDS = {'serendipity-table-v1': 'surface_point',
                  'serendipity-room-team-v1': 'object_center',
                  'serendipity-flat-table-v1': 'object_center'}
OUTCOMES = {'localized', 'target_not_visible', 'go2_foot_ik_failed', 'go2_lost_balance', 'unsupported_grasp_object'}


def _world_point(xyz):
    if not isinstance(xyz, list) or len(xyz) != 3 or not all(
        type(value) in (int, float) and isfinite(value) for value in xyz
    ):
        raise ValueError('world point must contain three finite XYZ coordinates in meters')


def validate(message):
    """Check the contract before a consumer uses a reported point."""
    if not isinstance(message, dict) or type(message.get('schema_version')) is not int or message['schema_version'] != VERSION:
        raise ValueError('unsupported handoff schema_version')
    if message.get('frame_id') != FRAME or message.get('units') != 'm':
        raise ValueError('handoff expects world-frame XYZ in meters')
    scene_id = message.get('scene_id')
    if (message.get('source_robot') != 'go2' or not isinstance(scene_id, str) or scene_id not in POSITION_KINDS
            or type(message.get('scene_seed')) is not int or message['scene_seed'] < 0):
        raise ValueError('handoff expects Go2 and a valid shared scene seed')
    status = message.get('status')
    if not isinstance(status, str) or status not in OUTCOMES:
        raise ValueError('unknown handoff status')
    for label in ('object_id', 'destination_id'):
        if not isinstance(message.get(label), str) or not message[label].strip():
            raise ValueError(f'{label} must be a nonempty string')
    _world_point(message.get('destination_xyz_m'))
    if 'observation' not in message or 'object_xyz_m' not in message:
        raise ValueError('handoff requires observation and object_xyz_m fields')
    observation = message['observation']
    point = message['object_xyz_m']
    if status == 'localized':
        _world_point(point)
        if (not isinstance(observation, dict) or observation.get('position_kind') != POSITION_KINDS[scene_id]
                or not isinstance(observation.get('method'), str) or not observation['method']
                or type(observation.get('capture_index')) is not int or observation['capture_index'] < 0):
            raise ValueError('localized handoff requires an observation record for the scene')
    elif point is not None or observation is not None:
        raise ValueError('failed handoff must have null object coordinates and observation')
    return message


def from_scout(seed, target, destination, destination_xyz, report, foot_failure, *,
               scene_id='serendipity-table-v1', position_kind='surface_point', failure_status='target_not_visible'):
    status = 'localized' if report is not None else failure_status
    if foot_failure:
        status = 'go2_foot_ik_failed'
    point = report['xyz_m'] if status == 'localized' else None
    observation = None if point is None else {
        'position_kind': position_kind, 'capture_index': report['capture_index'], 'method': report['method'],
    }
    return validate({'schema_version': VERSION, 'source_robot': 'go2',
                     'scene_id': scene_id, 'scene_seed': seed,
                     'frame_id': FRAME, 'units': 'm', 'status': status, 'object_id': target,
                     'object_xyz_m': point, 'destination_id': destination,
                     'destination_xyz_m': list(destination_xyz), 'observation': observation})
