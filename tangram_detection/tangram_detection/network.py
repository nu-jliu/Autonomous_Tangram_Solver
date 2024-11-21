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
print("CAE", output_tensor.shape)  # Should match (1, 1, 256, 256)


class Sampling(nn.Module):
    def __init__(self):
        super(Sampling, self).__init__()

    def forward(self, z_mean, z_log_var):
        epsilon = torch.randn_like(z_mean)
        return z_mean + epsilon * torch.exp(0.5 * z_log_var)


class VAE(nn.Module):
    def __init__(self, input_channels=1, latent_dim=2):
        super(VAE, self).__init__()
        self.latent_dim = latent_dim

        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(
                input_channels,
                48,
                kernel_size=11,
                stride=1,
                padding=5,
                padding_mode="replicate",
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.Conv2d(
                48,
                48,
                kernel_size=9,
                stride=2,
                padding=4,
                padding_mode="replicate",
            ),
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
            ),
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
            ),
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
            ),
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
            ),
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
            ),
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
            ),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
        )

        # Latent space
        self.flatten = nn.Flatten()
        self.z_mean = nn.Linear(48 * 8 * 8, latent_dim)
        self.z_log_var = nn.Linear(48 * 8 * 8, latent_dim)
        self.sampling = Sampling()

        # Decoder
        self.decoder_input = nn.ConvTranspose2d(
            latent_dim,
            48,
            kernel_size=5,
            stride=2,
            padding=2,
            output_padding=1,
        )
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=7,
                stride=2,
                padding=3,
                output_padding=1,
            ),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=9,
                stride=2,
                padding=4,
                output_padding=1,
            ),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
            nn.ConvTranspose2d(
                48,
                48,
                kernel_size=11,
                stride=2,
                padding=5,
                output_padding=1,
            ),
            nn.Dropout(0.2),
            nn.BatchNorm2d(48),
            nn.LeakyReLU(0.2),
        )
        self.output_layer = nn.Conv2d(48, input_channels, kernel_size=1)
        self.final_activation = nn.Softplus()

    def forward(self, x):
        # Encode
        x_encoded = self.encoder(x)
        x_flattened = self.flatten(x_encoded)

        # Latent space
        z_mean = self.z_mean(x_flattened)
        z_log_var = self.z_log_var(x_flattened)
        z = self.sampling(z_mean, z_log_var)

        # Decode
        z = z.view(z.size(0), z.size(1), 1, 1)  # Reshape for ConvTranspose2d
        x_decoded = self.decoder_input(z)
        x_decoded = self.decoder(x_decoded)
        x_out = self.output_layer(x_decoded)
        return self.final_activation(x_out), z_mean, z_log_var


# vae = VAE(input_channels=1)
# input_tensor = torch.randn(1, 1, 256, 256)
# output_tensor = vae(input_tensor)
# print("VAE", output_tensor.shape)  # Should match (1, 1, 256, 256)


class UNet(nn.Module):
    def __init__(self, input_channels, output_channels=1):
        super(UNet, self).__init__()

        # Contracting path
        self.conv1 = self._double_conv(input_channels, 64)
        self.pool1 = nn.MaxPool2d(kernel_size=2, stride=2)

        self.conv2 = self._double_conv(64, 128)
        self.pool2 = nn.MaxPool2d(kernel_size=2, stride=2)

        self.conv3 = self._double_conv(128, 256)
        self.pool3 = nn.MaxPool2d(kernel_size=2, stride=2)

        # Bottleneck
        self.bottleneck = self._double_conv(256, 512)

        # Expansive path
        self.upconv5 = nn.ConvTranspose2d(512, 256, kernel_size=2, stride=2)
        self.conv5 = self._double_conv(512, 256)

        self.upconv6 = nn.ConvTranspose2d(256, 128, kernel_size=2, stride=2)
        self.conv6 = self._double_conv(256, 128)

        self.upconv7 = nn.ConvTranspose2d(128, 64, kernel_size=2, stride=2)
        self.conv7 = self._double_conv(128, 64)

        # Final output layer
        self.final_conv = nn.Conv2d(64, output_channels, kernel_size=1)

    def _double_conv(self, in_channels, out_channels):
        """Two Conv2D layers with BatchNorm and ReLU activation."""
        return nn.Sequential(
            nn.Conv2d(in_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
            nn.Conv2d(out_channels, out_channels, kernel_size=3, padding=1),
            nn.BatchNorm2d(out_channels),
            nn.ReLU(inplace=True),
        )

    def forward(self, x):
        # Contracting path
        conv1 = self.conv1(x)
        pool1 = self.pool1(conv1)

        conv2 = self.conv2(pool1)
        pool2 = self.pool2(conv2)

        conv3 = self.conv3(pool2)
        pool3 = self.pool3(conv3)

        # Bottleneck
        bottleneck = self.bottleneck(pool3)

        # Expansive path
        up5 = self.upconv5(bottleneck)
        concat5 = torch.cat([conv3, up5], dim=1)
        conv5 = self.conv5(concat5)

        up6 = self.upconv6(conv5)
        concat6 = torch.cat([conv2, up6], dim=1)
        conv6 = self.conv6(concat6)

        up7 = self.upconv7(conv6)
        concat7 = torch.cat([conv1, up7], dim=1)
        conv7 = self.conv7(concat7)

        # Final output layer
        outputs = torch.sigmoid(self.final_conv(conv7))
        return outputs


# unet = UNet(input_channels=1)
# input_tensor = torch.randn(1, 1, 256, 256)
# output_tensor = unet(input_tensor)
# print("UNet", output_tensor.shape)  # Should match (1, 1, 256, 256)


class Discriminator(nn.Module):
    def __init__(self, image_shape):
        super(Discriminator, self).__init__()

        def conv_block(
            in_channels, out_channels, kernel_size, stride, padding, use_bn=True
        ):
            """Helper function to create a convolutional block with optional BatchNorm."""
            layers = [
                nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding),
                nn.LeakyReLU(0.2, inplace=True),
            ]
            if use_bn:
                layers.insert(1, nn.BatchNorm2d(out_channels))
            return nn.Sequential(*layers)

        channels = image_shape[0] * 2  # Source and target images are concatenated
        self.model = nn.Sequential(
            conv_block(channels, 64, kernel_size=4, stride=2, padding=1, use_bn=False),
            conv_block(64, 128, kernel_size=4, stride=2, padding=1),
            conv_block(128, 128, kernel_size=4, stride=2, padding=1),
            conv_block(128, 256, kernel_size=4, stride=2, padding=1),
            conv_block(256, 256, kernel_size=4, stride=2, padding=1),
            conv_block(256, 512, kernel_size=4, stride=2, padding=1),
            conv_block(512, 512, kernel_size=4, stride=1, padding=1),
            nn.Conv2d(512, 1, kernel_size=4, stride=1, padding=1),
            nn.Sigmoid(),  # PatchGAN output
        )

    def forward(self, src_image, target_image):
        # Concatenate source and target images along the channel dimension
        x = torch.cat((src_image, target_image), dim=1)
        return self.model(x)


# discriminator = Discriminator(image_shape=(1, 256, 256))
# input_tensor = torch.randn(1, 1, 256, 256)
# output_tensor = torch.randn(1, 1, 256, 256)

# output = discriminator(input_tensor, output_tensor)
# print("Discriminator", output.shape)  # Should match (1, 1, 256, 256)
