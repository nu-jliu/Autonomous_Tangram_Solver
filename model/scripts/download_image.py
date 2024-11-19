import paramiko
from scp import SCPClient
import os
import argparse
import time

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.join(FILE_DIR, "..", "test")
HOME_DIR = os.environ["HOME"]


def download_file_with_key(
    host,
    port,
    username,
    private_key_path,
    remote_file_path,
    local_file_path,
):
    try:
        # Load the private key
        private_key = paramiko.Ed25519Key.from_private_key_file(private_key_path)

        # Establish an SSH connection
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(hostname=host, port=port, username=username, pkey=private_key)

        # Use SCP to download the file
        with SCPClient(ssh.get_transport()) as scp:
            scp.get(remote_file_path, local_file_path)

        print(f"File downloaded successfully to {local_file_path}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print(f"Closing ssh connection to {host}\n")
        ssh.close()


# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--host",
        default="sheep.mech.northwestern.edu",
        help="Host name of the server",
    )
    parser.add_argument(
        "--username",
        default="jingkun",
        help="Username of the ssh server",
    )
    parser.add_argument(
        "--private-key",
        default="id_ed25519",
        help="File name of the private key",
    )
    parser.add_argument(
        "--filename",
        default="output_image.jpg",
        help="Output filename",
    )

    args = parser.parse_args()

    host = args.host
    username = args.username
    private_key_path = os.path.join(HOME_DIR, ".ssh", args.private_key)
    output_file = args.filename
    output_path = os.path.join(TEST_DIR, output_file)

    while True:

        download_file_with_key(
            host=host,
            port=22,
            username=username,
            private_key_path=private_key_path,  # Path to your private key
            remote_file_path="/home/jingkun/Final_Project/src/Autonomous_Tangram_Solver/model/test/output_test.jpg",
            local_file_path=output_path,
        )

        download_file_with_key(
            host=host,
            port=22,
            username=username,
            private_key_path=private_key_path,  # Path to your private key
            remote_file_path="/home/jingkun/Final_Project/src/Autonomous_Tangram_Solver/model/test/train_loss.png",
            local_file_path=os.path.join(TEST_DIR, "train_loss.png"),
        )

        # time.sleep(1)
