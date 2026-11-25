"""Exercise upstream environment entry points in their own Python installations."""
import argparse
import json


def maniskill(task, seed, steps):
    import gymnasium as gym
    import mani_skill.envs  # Registers the official Gymnasium environment IDs.

    env = gym.make(task, obs_mode='state', render_mode=None, num_envs=1, sim_backend='cpu')
    try:
        env.reset(seed=seed)
        env.action_space.seed(seed)
        for index in range(steps):
            _, _, terminated, truncated, info = env.step(env.action_space.sample())
            if bool(terminated) or bool(truncated):
                break
        success = info.get('success')
        return {'environment': task, 'policy': 'random', 'steps': index + 1,
                'success': bool(success) if success is not None else None}
    finally:
        env.close()


def libero(suite):
    from libero.libero import benchmark

    tasks = benchmark.get_benchmark(suite)()
    return {'suite': suite, 'tasks': [tasks.get_task(i).language for i in range(tasks.get_num_tasks())]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('backend', choices=['maniskill', 'libero'])
    parser.add_argument('task')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--steps', type=int, choices=range(1, 101), default=5, metavar='1-100')
    args = parser.parse_args()
    result = maniskill(args.task, args.seed, args.steps) if args.backend == 'maniskill' else libero(args.task)
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
