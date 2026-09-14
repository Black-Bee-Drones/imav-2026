import subprocess

def get_camera_index(camera_name: str) -> int:
    cmd = f'v4l2-ctl --list-devices | grep -i "{camera_name}" -A 4 | grep -o "video[0-9]\\+" | head -n 1 | grep -o "[0-9]\\+"'

    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    output = result.stdout.strip()
    if not output:
        print(f"Warning: Camera '{camera_name}' not found.")
        return -1

    return int(output)

def main() -> None:
    webcam_id = get_camera_index('WebCam')
    imx_id    = get_camera_index('IMX')
    c920_id   = get_camera_index('C920')
    d435i_id  = get_camera_index('RealSense')

    print(f'WebCam: {webcam_id}\nIMX: {imx_id}\nC920: {c920_id}\nRealSense: {d435i_id}')

if __name__ == '__main__':
    main()
