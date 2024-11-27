import paramiko
from scp import SCPClient
import os
import argparse
import time

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.join(FILE_DIR, "..", "test")
MODEL_DIR = os.path.join(FILE_DIR, "..", "model")
HOME_DIR = os.environ["HOME"]

REMOTE_DIR = os.path.join(HOME_DIR, "Final_Project/src/Autonomous_Tangram_Solver/model")
REMOTE_TEST_DIR = os.path.join(REMOTE_DIR, "test")
REMOTE_MODEL_DIR = os.path.join(REMOTE_DIR, "model")


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
        "--device",
        default="sheep",
        choices=["lamb", "sheep"],
        help="The device to download model from",
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
        "--model",
        default="tangram_cae.pth",
        help="Output model filename",
    )
    parser.add_argument(
        "--csv",
        default="epoch_loss_cae.csv",
        help="Output CSV filename",
    )

    args = parser.parse_args()

    device = args.device
    host = f"{device}.mech.northwestern.edu"
    port = 22
    username = args.username
    private_key_path = os.path.join(HOME_DIR, ".ssh", args.private_key)
    csv_file = args.csv
    model_file = args.model

    while True:

        download_file_with_key(
            host=host,
            port=port,
            username=username,
            private_key_path=private_key_path,  # Path to your private key
            remote_file_path=os.path.join(REMOTE_TEST_DIR, csv_file),
            local_file_path=os.path.join(TEST_DIR, csv_file),
        )

        download_file_with_key(
            host=host,
            port=port,
            username=username,
            private_key_path=private_key_path,  # Path to your private key
            remote_file_path=os.path.join(REMOTE_MODEL_DIR, model_file),
            local_file_path=os.path.join(MODEL_DIR, model_file),
        )

        time.sleep(2)
