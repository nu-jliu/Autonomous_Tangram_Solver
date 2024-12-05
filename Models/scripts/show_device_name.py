import os
import subprocess


def get_video_devices():
    devices = [
        f"/dev/{device}" for device in os.listdir("/dev") if device.startswith("video")
    ]
    # print(devices)
    device_info = []

    for device in devices:
        try:
            # Run v4l2-ctl to get device name
            output = subprocess.check_output(
                ["v4l2-ctl", "--device", device, "--all"], text=True
            )
            # print(device)
            # print(output)
            for line in output.splitlines():
                if "Card type" in line:
                    name = line.split(":", 1)[1].strip()
                    device_info.append((device, name))
                    break
        except subprocess.CalledProcessError as e:
            print(f"Failed to get info for {device}: {e}")
        except FileNotFoundError:
            print("v4l2-ctl not found. Please install v4l-utils.")
            return []

    return device_info


if __name__ == "__main__":
    devices = get_video_devices()
    for device, name in devices:
        print(f"{device}: {name}")
