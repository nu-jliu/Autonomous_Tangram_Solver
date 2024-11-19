from network import CAE

import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim

from torchvision import transforms
from PIL import Image  # For loading images

import os  # For file and directory management

FILE_DIR = os.path.dirname(os.path.abspath(__file__))

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

# Transform: Resize and Normalize Images
transform = transforms.Compose([
    transforms.Resize((256, 256)),
    transforms.ToTensor(),  # Convert to tensor and normalize to [0, 1]
])

# Dataset and DataLoader
input_dir = os.path.join(FILE_DIR, "dataset", "input_cae")
target_dir = os.path.join(FILE_DIR, "dataset", "output_cae")
dataset = BinaryImageDataset(input_dir, target_dir, transform=transform)
dataloader = DataLoader(dataset, batch_size=32, shuffle=True)

def binary_wmae_loss(c=5):
    def loss(y_true, y_pred):
        # Normalize predictions (apply sigmoid for probabilistic output)
        y_pred = torch.sigmoid(y_pred)

        # Delta
        delta = y_true - y_pred

        # Weights
        greater = (delta > 0).float()
        less = (delta < 0).float()
        w = F.normalize(greater * y_true, p=2, dim=-1) * c + less

        return torch.mean(torch.abs(w * delta))
    return loss

criterion = binary_wmae_loss(c=5)


num_epochs = 10
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
model = CAE(input_channels=1)  # Replace `CAE` with your model class

# Assume `model` is your defined PyTorch model
optimizer = optim.Adam(model.parameters(), lr=1e-4)  # Learning rate = 0.0001



for epoch in range(num_epochs):
    model.train()
    epoch_loss = 0

    for i, (inputs, targets) in enumerate(dataloader):
        inputs, targets = inputs.to(device), targets.to(device)

        # Forward Pass
        outputs = model.forward(inputs)
        outputs = outputs.squeeze(1)  # Remove channel dimension

        loss = criterion(outputs, targets)

        # Backward Pass and Optimization
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        epoch_loss += loss.item()

        if (i + 1) % 10 == 0:
            print(f"Batch [{i+1}/{len(dataloader)}], Loss: {loss.item():.4f}")

    print(f"Epoch [{epoch+1}/{num_epochs}], Loss: {epoch_loss / len(dataloader):.4f}")


