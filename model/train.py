import os
from PIL import Image

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms

from tqdm import tqdm

from network import CAE
from wmae import wmae_loss

FILE_DIR = os.path.dirname(os.path.realpath(__file__))


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
    # Transform: Resize and Normalize Images
    transform = transforms.Compose(
        [
            transforms.Resize((256, 256)),
            transforms.ToTensor(),  # Convert to tensor and normalize to [0, 1]
        ]
    )

    # Dataset and DataLoader
    input_dir = os.path.join(FILE_DIR, "dataset", "input_cae")
    target_dir = os.path.join(FILE_DIR, "dataset", "output_cae")
    dataset = BinaryImageDataset(input_dir, target_dir, transform=transform)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

    model = CAE(input_channels=1)  # For RGB images
    model = model.to("cuda")  # Move model to GPU if available

    criterion = wmae_loss(c=5)  # Weighted Mean Absolute Error
    optimizer = torch.optim.Adam(model.parameters(), lr=1e-4)

    num_epochs = 100
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0

        for i, (inputs, targets) in enumerate(dataloader):
            inputs, targets = inputs.to(device), targets.to(device)

            # Forward Pass
            outputs = model(inputs)
            outputs = outputs.squeeze(1)  # Remove channel dimension

            loss = criterion(outputs, targets)

            # Backward Pass and Optimization
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            epoch_loss += loss.item()

            if (i + 1) % 10 == 0:
                print(f"Batch [{i+1}/{len(dataloader)}], Loss: {loss.item():.4f}")

        print(
            f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss / len(dataloader):.4f}"
        )

    torch.save(model.state_dict(), os.path.join(FILE_DIR, "model", "tangram.pth"))
    model.eval()
