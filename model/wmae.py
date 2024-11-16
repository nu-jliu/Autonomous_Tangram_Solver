import torch
import torch.nn.functional as F


def wmae_loss(c=5):
    def loss(y_true, y_pred):
        # Normalization
        y_true_norm = (255 - y_true) / 255
        y_pred_norm = (255 - y_pred) / 255

        # Delta
        delta = y_true_norm - y_pred_norm

        # Weighted Absolute Error
        greater = (delta > 0).float()
        less = (delta < 0).float()
        w = F.normalize(greater * y_true_norm, p=2, dim=-1) * c + less
        return torch.mean(torch.abs(w * delta))

    return loss
