"""GRU motion prediction network.

Architecture (matching proposal report):
- Input: sliding window of L=10 frames, each 6-dim [dx, dy, dvx, dvy, w, h]
- 1-layer GRU (hidden_dim=64) + 1 FC layer
- Output: next-frame displacement [dx_hat, dy_hat]
"""

import torch
import torch.nn as nn


class GRUPredictor(nn.Module):
    def __init__(self, input_dim=6, hidden_dim=64, output_dim=2, num_layers=1):
        super().__init__()
        self.hidden_dim = hidden_dim
        self.num_layers = num_layers

        self.gru = nn.GRU(
            input_size=input_dim,
            hidden_size=hidden_dim,
            num_layers=num_layers,
            batch_first=True,
        )
        self.fc = nn.Linear(hidden_dim, output_dim)

    def forward(self, x, h0=None):
        """
        Args:
            x: (batch, seq_len, input_dim) incremental features
            h0: optional initial hidden state (num_layers, batch, hidden_dim)
        Returns:
            pred: (batch, output_dim) predicted displacement
            hn: (num_layers, batch, hidden_dim) final hidden state
        """
        if h0 is None:
            h0 = torch.zeros(
                self.num_layers, x.size(0), self.hidden_dim, device=x.device
            )
        out, hn = self.gru(x, h0)
        pred = self.fc(out[:, -1, :])  # use last time step
        return pred, hn
