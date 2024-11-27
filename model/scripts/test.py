import torch
import os

import cv2
from torchvision import transforms
from PIL import Image
import matplotlib.pyplot as plt
from torchvision.utils import save_image

from network import CAE, Sampling, VAE, UNet, Discriminator
import time

import pandas as pd
import numpy as np

import argparse

FILE_DIR = os.path.dirname(os.path.abspath(__file__))
TEST_DIR = os.path.join(FILE_DIR, "..", "test")
MODEL_DIR = os.path.join(FILE_DIR, "..", "model")
DATASET_DIR = os.path.join(FILE_DIR, "..", "dataset")

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--filename",
        default="23.png",
        help="The file name for the input image for interence",
    )

    args = parser.parse_args()
    test_filename = args.filename

    while True:

        try:
            # Load model
            print("load")
            model = CAE(input_channels=1)  # For grayscale
            model.load_state_dict(
                torch.load(
                    os.path.join(MODEL_DIR, "tangram_cae.pth"),
                    weights_only=True,
                )
            )

            model.eval()
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            model = model.to(device)

            # Preprocess input image
            transform = transforms.Compose(
                [
                    transforms.Resize((256, 256)),  # Adjust size
                    transforms.ToTensor(),  # Convert to tensor
                ]
            )

            image_path = os.path.join(DATASET_DIR, "input_test", test_filename)
            input_image = Image.open(image_path).convert("L")  # Convert to grayscale
            input_tensor = transform(input_image).unsqueeze(0)  # Add batch dimension
            input_tensor = (input_tensor > 0.5).float()
            input_tensor = input_tensor.to(device)

            input_array = input_tensor.squeeze(0).cpu().numpy()
            input_array = (input_array * 255).astype(np.uint8)

            if input_array.ndim > 2:
                input_array = input_array[0]

            # print("start")
            # edges = cv2.Canny(input_array, 50.0, 150.0)
            # contours, _ = cv2.findContours(
            #     edges,
            #     cv2.RETR_EXTERNAL,
            #     cv2.CHAIN_APPROX_SIMPLE,
            # )

            # for contour in contours:
            #     eplison = 0.02 * cv2.arcLength(contour, True)
            #     contour_approx = cv2.approxPolyDP(contour, eplison, True)
            #     print(cv2.contourArea(contour))

            print(input_array.shape)
            print(np.sum((input_array / 255).astype(bool)))

            # cv2.imshow("Test", input_array)
            # cv2.waitKey(0)

            cv2.imwrite("test.png", input_array)

            # Perform inference
            with torch.no_grad():
                output = model.forward(input_tensor)

            # Postprocess the output
            output_image = output.squeeze(0).cpu()  # Remove batch dimension
            output_image = (output_image > 0.5).float()

            # Visualize output
            # plt.imshow(output_image.squeeze(0), cmap="gray")
            # plt.axis("off")
            # plt.show()

            # Save output
            save_image(output_image, os.path.join(TEST_DIR, "output_image_cae.png"))

            try:
                df = pd.read_csv(os.path.join(TEST_DIR, "epoch_loss_cae.csv"))
                losses = df["loss"].to_list()

                plt.cla()
                plt.plot(np.arange(0, len(losses), 1), losses)
                plt.title("Loss at each Epoch")
                plt.xlabel("Epoch")
                plt.ylabel("Loss")
                plt.savefig(os.path.join(TEST_DIR, "epoch_loss_cae.png"))
            except FileExistsError:
                print("File not found")

            print("Interenced image and loss image saved")
        except Exception as e:
            print(f"Received exception {e}")

        time.sleep(1)
