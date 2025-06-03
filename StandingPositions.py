import torch
import numpy as np
import torch.nn as nn

# Actor MLP structure (as printed in your play log)
class Actor(nn.Module):
    def __init__(self, obs_dim=42, action_dim=10):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 128),
            nn.ELU(),
            nn.Linear(128, 128),
            nn.ELU(),
            nn.Linear(128, 128),
            nn.ELU(),
            nn.Linear(128, action_dim)
        )

    def forward(self, x):
        return self.net(x)

# Load model
model_path = "scripts/rsl_rl/logs/rsl_rl/bdx_flat/2025-05-15_13-16-13/model_999.pt"
checkpoint = torch.load(model_path, map_location="cpu")

# Rebuild actor and load weights
actor = Actor()
actor.load_state_dict(checkpoint["model_state_dict"], strict=False)
actor.eval()

# Dummy observation (zeros)
obs = np.zeros((1, 42), dtype=np.float32)
obs_tensor = torch.tensor(obs)

# Get action
with torch.no_grad():
    action = actor(obs_tensor).squeeze().numpy()

print("Standing joint positions:")
print(action)
