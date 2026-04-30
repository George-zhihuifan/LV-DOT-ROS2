import argparse
import subprocess


TOPICS = [
    '/rgbd_camera/image',
    '/rgbd_camera/camera_info',
    '/rgbd_camera/depth_image',
    '/rgbd_camera/points',
]


def main() -> None:
    parser = argparse.ArgumentParser(description='Check whether expected ROS2 topics are visible.')
    parser.parse_args()

    result = subprocess.run(
        ['ros2', 'topic', 'list'],
        check=True,
        capture_output=True,
        text=True,
    )
    available = set(line.strip() for line in result.stdout.splitlines() if line.strip())
    missing = [topic for topic in TOPICS if topic not in available]

    for topic in TOPICS:
        print(f'{topic}: {"ok" if topic in available else "missing"}')

    if missing:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
