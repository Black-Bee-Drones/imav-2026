import subprocess

def get_camera_index(camera_name: str) -> int:
    """
    Retrieves the V4L2 video device index for a specified camera name.

    This function executes system shell commands to parse the output of
    'v4l2-ctl --list-devices' and extracts the first matching video
    node index (e.g., returns 0 for '/dev/video0').

    Parameters
    ----------
    camera_name : str
        The name or partial name of the camera to search for.

    Returns
    -------
    int
        The integer index of the video device if found, or -1 if the
        camera is not found or the command yields no output.

    Notes
    -----
    This function is Linux-specific and requires the 'v4l2-utils' package
    to be installed on the system to run the 'v4l2-ctl' command.
    """

    cmd = f'v4l2-ctl --list-devices | grep -i "{camera_name}" -A 4 | grep -o "video[0-9]\\+" | head -n 1 | grep -o "[0-9]\\+"'

    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)

    output = result.stdout.strip()
    if not output:
        print(f"Warning: Camera '{camera_name}' not found.")
        return -1

    print(f"{camera_name} at idx: {output}")

    return int(output)

def main() -> None:
    webcam_id = get_camera_index('WebCam')
    imx_id    = get_camera_index('Arducam')
    c920_id   = get_camera_index('C920')
    d435i_id  = get_camera_index('RealSense')

    print(f'WebCam: {webcam_id}\nArducam: {imx_id}\nC920: {c920_id}\nRealSense: {d435i_id}')

if __name__ == '__main__':
    main()
