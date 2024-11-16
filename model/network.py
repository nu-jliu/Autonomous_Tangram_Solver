import torch
import torch.nn as nn
import torch.nn.functional as F


# Sampling Layer for VAE
class Sampling(nn.Module):
    def forward(self, z_mean, z_log_var):
        std = torch.exp(0.5 * z_log_var)
        eps = torch.randn_like(std)
        return z_mean + eps * std


# Convolutional Autoencoder (CAE)
class CAE(nn.Module):
    def __init__(self, input_channels):
        super(CAE, self).__init__()
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(input_channels, 48, kernel_size=11, stride=1, padding=5),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(48, 48, kernel_size=9, stride=2, padding=4),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(48, 48, kernel_size=7, stride=2, padding=3),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(48, 48, kernel_size=5, stride=2, padding=2),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
        )
        # Decoder
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(
                48, 48, kernel_size=5, stride=2, padding=2, output_padding=1
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48, 48, kernel_size=7, stride=2, padding=3, output_padding=1
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48, 48, kernel_size=9, stride=2, padding=4, output_padding=1
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(48, input_channels, kernel_size=11, stride=1, padding=5),
            nn.BatchNorm2d(input_channels),
            nn.Softplus(),
        )

    def forward(self, x):
        encoded = self.encoder(x)
        decoded = self.decoder(encoded)
        return decoded


# Variational Autoencoder (VAE)
class VAE(nn.Module):
    def __init__(self, input_channels, latent_dim):
        super(VAE, self).__init__()
        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(input_channels, 48, kernel_size=11, stride=1, padding=5),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(48, 48, kernel_size=9, stride=2, padding=4),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(48, 48, kernel_size=7, stride=2, padding=3),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
        )
        self.fc_mean = nn.Linear(
            48 * 4 * 4, latent_dim
        )  # Adjust shape for your input size
        self.fc_log_var = nn.Linear(48 * 4 * 4, latent_dim)
        self.sampling = Sampling()

        # Decoder
        self.decoder_input = nn.Linear(latent_dim, 48 * 4 * 4)
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(
                48, 48, kernel_size=7, stride=2, padding=3, output_padding=1
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(48, input_channels, kernel_size=11, stride=1, padding=5),
            nn.BatchNorm2d(input_channels),
            nn.Softplus(),
        )

    def encode(self, x):
        x = self.encoder(x)
        x = torch.flatten(x, start_dim=1)
        z_mean = self.fc_mean(x)
        z_log_var = self.fc_log_var(x)
        return z_mean, z_log_var

    def decode(self, z):
        z = self.decoder_input(z)
        z = z.view(-1, 48, 4, 4)  # Adjust to match input dimensions
        return self.decoder(z)

    def forward(self, x):
        z_mean, z_log_var = self.encode(x)
        z = self.sampling(z_mean, z_log_var)
        return self.decode(z)


# U-Net
class UNet(nn.Module):
    def __init__(self, input_channels, output_channels):
        super(UNet, self).__init__()
        self.down1 = self.conv_block(input_channels, 64)
        self.down2 = self.conv_block(64, 128)
        self.down3 = self.conv_block(128, 256)

        self.bottleneck = self.conv_block(256, 512)

        self.up3 = self.conv_block(512 + 256, 256)
        self.up2 = self.conv_block(256 + 128, 128)
        self.up1 = self.conv_block(128 + 64, 64)

        self.final = nn.Conv2d(64, output_channels, kernel_size=1)

    def conv_block(self, in_channels, out_channels):
        return nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.BatchNorm2d(out_channels),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1),
            nn.ReLU(inplace=True),
            nn.BatchNorm2d(out_channels),
        )

    def forward(self, x):
        d1 = self.down1(x)
        d2 = self.down2(F.max_pool2d(d1, 2))
        d3 = self.down3(F.max_pool2d(d2, 2))

        b = self.bottleneck(F.max_pool2d(d3, 2))

        u3 = self.up3(
            torch.cat([F.interpolate(b, scale_factor=2, mode="bilinear"), d3], dim=1)
        )
        u2 = self.up2(
            torch.cat([F.interpolate(u3, scale_factor=2, mode="bilinear"), d2], dim=1)
        )
        u1 = self.up1(
            torch.cat([F.interpolate(u2, scale_factor=2, mode="bilinear"), d1], dim=1)
        )

        return torch.sigmoid(self.final(u1))


# Discriminator
class Discriminator(nn.Module):
    def __init__(self, input_channels):
        super(Discriminator, self).__init__()
        self.model = nn.Sequential(
            nn.Conv2d(input_channels * 2, 64, kernel_size=4, stride=2, padding=1),
            nn.LeakyReLU(0.2),
            nn.Conv2d(64, 128, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(128),
            nn.LeakyReLU(0.2),
            nn.Conv2d(128, 256, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(256),
            nn.LeakyReLU(0.2),
            nn.Conv2d(256, 512, kernel_size=4, stride=2, padding=1),
            nn.BatchNorm2d(512),
            nn.LeakyReLU(0.2),
            nn.Conv2d(512, 1, kernel_size=4, padding=1),
            nn.Sigmoid(),
        )

    def forward(self, src, target):
        x = torch.cat([src, target], dim=1)
        return self.model(x)
