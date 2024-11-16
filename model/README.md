# TangramDeep
## Description
The Tangram is a dissection puzzle consisting of seven polygonal pieces that can be arranged to form various patterns. Solving the Tangram poses a challenge as it is an irregular shape packing problem recognized as NP-hard. This paper delves into the application of four distinct deep-learning architectures—Convolutional Autoencoder, Variational Autoencoder, U-Net, and Generative Adversarial Network—tailored for solving Tangram puzzles. The primary focus is on understanding the capabilities of these architectures in capturing the intricate spatial relationships inherent in Tangram configurations.

Our experiments reveal that the Generative Adversarial Network exhibits competitive performance when compared to the other architectures and converges notably faster. We also discover that traditional evaluation metrics based on pixel accuracy often fall short in adequately assessing the visual quality of the generated Tangram solutions. To address this limitation, we introduce a novel loss function grounded in a Weighted Mean Absolute Error. This function prioritizes pixels representing inter-piece sections over those covered by individual pieces, offering a more nuanced evaluation of the generated solutions.

Furthermore, we extend this novel loss function to propose an innovative evaluation metric. This metric provides a more suitable measure for assessing Tangram solutions compared to traditional metrics. The insights gained from this investigation advance our understanding of the capabilities of artificial intelligence in tackling complex geometrical problem domains, particularly in the context of irregular shape-packing problems.

## Repository Contents
- dataset.zip: Contains the dataset utilized for training and evaluation. Please unzip this file before running any code.

- architectures.py: Python script housing the implementations of CAE, VAE, U-Net, and GAN for solving Tangram puzzles.

- loss_wmae.py: Implementation of the custom loss function based on Weighted Mean Absolute Error, introduced in the paper.

- requirements.txt: Library requirements for reproducing the reported results.

## Getting Started
### Prerequisites
Ensure you have the required dependencies installed by referring to the list provided in the requirements.txt file.

```
pip install -r requirements.txt
```
### Installation
Clone the repository.

```
git clone https://github.com/your-username/TangramDeep.git
cd TangramDeep
```

### Usage
Use the custom loss function in your code by importing loss_wmae.
```
from wmae import wmae_loss

# ... (your code)
model.compile(optimizer='adam', loss=wmae_loss)
# ... (continue with your code)
```

## Contact
For questions or information, contact Fernanda Miyuki Yamada at y2140014@edu.cc.uec.ac.jp.

Happy coding with TangramDeep!

