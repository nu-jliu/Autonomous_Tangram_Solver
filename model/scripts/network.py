import torch
import torch.nn as nn
import torch.nn.functional as F


class CAE(nn.Module):
    def __init__(self, input_channels=1):
        super(CAE, self).__init__()
        self.encoder = nn.Sequential(
            nn.Conv2d(
                input_channels,
                48,
                kernel_size=11,
                stride=1,
                padding=5,
                padding_mode="replicate",
            ),  # Input -> (256, 256)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=9,
                stride=2,
                padding=4,
                padding_mode="replicate",
            ),  # Downsample -> (128, 128)
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=9,
                stride=2,
                padding=4,
                padding_mode="replicate",
            ),  # Downsample -> (64, 64)
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=7,
                stride=2,
                padding=3,
                padding_mode="replicate",
            ),  # Downsample -> (32, 32)
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=5,
                stride=2,
                padding=2,
                padding_mode="replicate",
            ),  # Downsample -> (16, 16)
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=3,
                stride=2,
                padding=1,
                padding_mode="replicate",
            ),  # Downsample -> (8, 8)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
        )

        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=3,
                stride=2,
                padding=1,
                output_padding=1,
                # padding_mode="replicate",
            ),  # Upsample -> (16, 16)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=5,
                stride=2,
                padding=2,
                output_padding=1,
                # padding_mode="replicate",
            ),  # Upsample -> (32, 32)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=7,
                stride=2,
                padding=3,
                output_padding=1,
                # padding_mode="replicate",
            ),  # Upsample -> (64, 64)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=9,
                stride=2,
                padding=4,
                output_padding=1,
                # padding_mode="replicate",
            ),  # Upsample -> (128, 128)
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                input_channels,
                kernel_size=11,
                stride=2,
                padding=5,
                output_padding=1,
                # padding_mode="replicate",
            ),  # Upsample -> (256, 256)
            nn.BatchNorm2d(input_channels),
            nn.Softplus(),  # Smooth output
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded


# Instantiate CAE
cae = CAE(input_channels=1)
input_tensor = torch.randn(1, 1, 256, 256)
output_tensor = cae(input_tensor)
print(output_tensor.shape)  # Should match (1, 1, 256, 256)
