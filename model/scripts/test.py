import torch
import os
from torchvision import transforms
from PIL import Image
import matplotlib.pyplot as plt
from torchvision.utils import save_image

from network import CAE

FILE_DIR = os.path.dirname(os.path.abspath(__file__))

if __name__ == "__main__":

    # Load model
    model = CAE(input_channels=1)  # For grayscale
    model.load_state_dict(
        torch.load(os.path.join(FILE_DIR, "..", "model", "tangram.pth"))
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

    image_path = os.path.join(FILE_DIR, "..", "dataset", "input_test", "23.png")
    input_image = Image.open(image_path).convert("L")  # Convert to grayscale
    input_tensor = transform(input_image).unsqueeze(0)  # Add batch dimension
    input_tensor = (input_tensor > 0.5).float()
    input_tensor = input_tensor.to(device)

    # Perform inference
    with torch.no_grad():
        output = model.forward(input_tensor)

    # Postprocess the output
    output_image = output.squeeze(0).cpu()  # Remove batch dimension
    output_image = (output_image > 0.5).float()

    # Visualize output
    plt.imshow(output_image.squeeze(0), cmap="gray")
    plt.axis("off")
    plt.show()

    # Save output
    save_image(output_image, "output_test.jpg")
