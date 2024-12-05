import os
from PIL import Image
import numpy as np

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision.utils import save_image
from torchvision import transforms

from tqdm import tqdm

from network import CAE
from wmae import wmae_loss
import matplotlib.pyplot as plt
import argparse
import copy
import pandas as pd

FILE_DIR = os.path.dirname(os.path.realpath(__file__))
ROOT_DIR = os.path.join(FILE_DIR, "..")
DATASET_DIR = os.path.join(ROOT_DIR, "dataset")
MODEL_DIR = os.path.join(ROOT_DIR, "model")
TEST_DIR = os.path.join(ROOT_DIR, "test")


class BinaryImageDataset(Dataset):
    def __init__(self, input_dir, target_dir, transform=None):
        self.input_dir = input_dir
        self.target_dir = target_dir
        self.transform = transform
        self.input_images = sorted(os.listdir(input_dir))
        self.target_images = sorted(os.listdir(target_dir))

    def __len__(self):
        return len(self.input_images)

    def __getitem__(self, idx):
        input_path = os.path.join(self.input_dir, self.input_images[idx])
        target_path = os.path.join(self.target_dir, self.target_images[idx])

        input_image = Image.open(input_path).convert("L")  # Grayscale
        target_image = Image.open(target_path).convert("L")  # Grayscale

        if self.transform:
            input_image = self.transform(input_image)
            target_image = self.transform(target_image)

        # Binarize the images (pixel values: 0 or 1)
        input_image = (input_image > 0.5).float()
        target_image = (target_image > 0.5).float()

        # print(input_path)
        # print(target_path)
        # print("")

        return input_image, target_image


def initialize_weights(m):
    if isinstance(m, nn.Conv2d) or isinstance(m, nn.ConvTranspose2d):
        nn.init.kaiming_normal_(m.weight, nonlinearity="relu")
        if m.bias is not None:
            nn.init.constant_(m.bias, 0)
    elif isinstance(m, nn.BatchNorm2d):
        nn.init.constant_(m.weight, 1)
        nn.init.constant_(m.bias, 0)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--num-epoch", default=5000, help="Number of epochs")
    parser.add_argument(
        "--output-filename",
        default="output_test_cae.jpg",
        help="name of the output test file",
    )
    parser.add_argument(
        "--model-filename",
        default="tangram_cae",
        help="Name of the model file",
    )
    parser.add_argument("--lamb", action="store_true", help="Use lambda to run it")
    parser.add_argument(
        "--load",
        action="store_true",
        help="Load the pre-trained model",
    )
    parser.add_argument(
        "--loss-filename",
        default="epoch_loss_cae.csv",
        help="File name of the loss csv data for each epoch",
    )

    args = parser.parse_args()

    print(args)
    num_epochs = int(args.num_epoch)
    use_lamdba = args.lamb
    output_filename = os.path.join(TEST_DIR, args.output_filename)
    model_filename = os.path.join(MODEL_DIR, f"{args.model_filename}.pth")
    loss_filename = os.path.join(TEST_DIR, args.loss_filename)
    load = args.load

    batch_size = 128 if use_lamdba else 32

    # Transform: Resize and Normalize Images
    transform = transforms.Compose(
        [
            transforms.Resize((256, 256)),
            transforms.ToTensor(),  # Convert to tensor and normalize to [0, 1]
        ]
    )

    # Dataset and DataLoader
    input_dir = os.path.join(DATASET_DIR, "input_cae")
    target_dir = os.path.join(DATASET_DIR, "output_cae")
    dataset = BinaryImageDataset(input_dir, target_dir, transform=transform)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    model = CAE(input_channels=1)  # For Binary images
    if load:
        model.load_state_dict(torch.load(model_filename))

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.to(device)  # Move model to GPU if available

    criterion = wmae_loss(c=50)  # Weighted Mean Absolute Error
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    image_path = os.path.join(DATASET_DIR, "input_test", "23.png")
    input_image = Image.open(image_path).convert("L")  # Convert to grayscale
    input_tensor = transform(input_image).unsqueeze(0)  # Add batch dimension
    # input_tensor = (input_tensor > 0.5).float()
    input_tensor = input_tensor.to(device)

    # num_epochs = 100
    losses = []
    try:
        df = pd.read_csv(loss_filename)
        losses = df["loss"].to_list()
    except FileNotFoundError:
        print("File not found")

    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0

        # for i, (inputs, targets) in enumerate(dataloader):
        #     inputs, targets = inputs.to(device), targets.to(device)

        with tqdm(
            dataloader,
            unit="batch",
            colour="green",
        ) as tepoch:
            tepoch.set_description(f"Epoch {epoch+1}/{num_epochs}")

            for inputs, targets in tepoch:
                inputs, targets = inputs.to(device), targets.to(device)

                # Forward Pass
                outputs = model.forward(inputs)
                # outputs = outputs.squeeze(1)  # Remove channel dimension

                loss = criterion(targets, outputs)

                # Backward Pass and Optimization
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

                tepoch.set_postfix(loss=loss.item())
                epoch_loss += loss.item()

                # Perform inference
                model_ev = copy.deepcopy(model)
                torch.save(
                    model_ev.state_dict(),
                    os.path.join(MODEL_DIR, model_filename),
                )
                # model_ev.eval()
                # with torch.no_grad():
                #     output = model_ev.forward(input_tensor)

                # # Postprocess the output
                # output_image = output.squeeze(0).cpu()  # Remove batch dimension
                # output_image = (output_image > 0.5).float()

                # save_image(output_image, output_filename)

                # plt.cla()
                # plt.plot(np.arange(0, len(losses), 1), losses)
                # plt.savefig(os.path.join(TEST_DIR, "train_loss.png"))

                epoch_dict = {"loss": losses}
                df = pd.DataFrame(epoch_dict)
                df.to_csv(loss_filename)

                # if (i + 1) % 10 == 0:
                #     print(f"Batch [{i+1}/{len(dataloader)}], Loss: {loss.item():.4f}")
        losses.append(epoch_loss)

        # print(
        #     f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss / len(dataloader):.4f}"
        # )
        print("")

    torch.save(model.state_dict(), os.path.join(MODEL_DIR, model_filename))
    model.eval()

    # Preprocess input image
    transform = transforms.Compose(
        [
            transforms.Resize((256, 256)),  # Adjust size
            transforms.ToTensor(),  # Convert to tensor
        ]
    )

    plt.cla()
    plt.plot(np.arange(0, num_epochs, 1), losses)
    plt.savefig(os.path.join(TEST_DIR, "train_loss.png"))
