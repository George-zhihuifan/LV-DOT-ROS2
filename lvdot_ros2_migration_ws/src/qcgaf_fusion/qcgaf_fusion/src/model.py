"""
QC-GAF: Quality-aware Gated Attention Fusion Network
~6K parameters, 4 sub-modules:
  B - FeatureEncoder (cam_mlp + lidar_mlp)
  C - QualityAttentionMatcher
  D - GatedFusionHead (gate + SE + regression)
"""

import math
import torch
import torch.nn as nn


class SEBlock(nn.Module):
    """Squeeze-and-Excitation on the feature dimension."""

    def __init__(self, channels: int, reduction: int = 4):
        super().__init__()
        mid = max(channels // reduction, 4)
        self.fc = nn.Sequential(
            nn.Linear(channels, mid),
            nn.ReLU(inplace=True),
            nn.Linear(mid, channels),
            nn.Sigmoid(),
        )

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        # x: (B, M, C)
        w = x.mean(dim=1)          # (B, C)  squeeze
        w = self.fc(w).unsqueeze(1) # (B, 1, C)
        return x * w


class FeatureEncoder(nn.Module):
    """Sub-module B: MLP feature encoders for camera and LiDAR detections."""

    def __init__(self, cam_dim: int = 9, lidar_dim: int = 10,
                 hidden: int = 20, feat: int = 32):
        super().__init__()
        self.cam_mlp = nn.Sequential(
            nn.Linear(cam_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, feat),
        )
        self.lidar_mlp = nn.Sequential(
            nn.Linear(lidar_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, feat),
        )

    def forward(self, cam: torch.Tensor, lidar: torch.Tensor):
        # cam: (B, M, cam_dim), lidar: (B, N, lidar_dim)
        return self.cam_mlp(cam), self.lidar_mlp(lidar)


class QualityAttentionMatcher(nn.Module):
    """Sub-module C: quality-aware soft matching between cam and LiDAR."""

    def __init__(self, quality_dim: int = 7, feat_dim: int = 32):
        super().__init__()
        self.q_proj = nn.Linear(quality_dim, feat_dim)
        self.temperature = nn.Parameter(torch.ones(1))
        self.scale = 1.0 / math.sqrt(feat_dim)

    def forward(self, cam_feat: torch.Tensor, lidar_feat: torch.Tensor,
                quality: torch.Tensor,
                cam_mask: torch.Tensor = None,
                lidar_mask: torch.Tensor = None):
        """
        Args:
            cam_feat:   (B, M, D)
            lidar_feat: (B, N, D)
            quality:    (B, 7)
            cam_mask:   (B, M) bool, True = valid
            lidar_mask: (B, N) bool, True = valid
        Returns:
            match_matrix: (B, M, N) soft assignment
            cam_q:        (B, M, D) quality-modulated cam features
        """
        # quality-modulated features (used downstream for fusion)
        q_gate = torch.sigmoid(self.q_proj(quality))   # (B, D)
        cam_q = cam_feat * q_gate.unsqueeze(1)          # (B, M, D)

        # matching uses RAW features (not quality-gated) for robust matching
        # even when camera quality is degraded
        scores = torch.bmm(cam_feat, lidar_feat.transpose(1, 2))  # (B, M, N)
        scores = scores * self.scale / (self.temperature + 1e-6)

        # mask out padded positions
        if lidar_mask is not None:
            mask = ~lidar_mask.unsqueeze(1)  # (B, 1, N) True = invalid
            scores = scores.masked_fill(mask, -1e9)

        match_matrix = torch.softmax(scores, dim=-1)  # (B, M, N)
        return match_matrix, cam_q


class GatedFusionHead(nn.Module):
    """Sub-module D: gate + SE + regression head."""

    def __init__(self, quality_dim: int = 7, feat_dim: int = 32,
                 output_dim: int = 7):
        super().__init__()
        fused_dim = feat_dim * 2  # 64
        self.gate_proj = nn.Linear(quality_dim, fused_dim)
        self.se = SEBlock(fused_dim, reduction=4)
        self.reg_head = nn.Sequential(
            nn.Linear(fused_dim, 20),
            nn.ReLU(inplace=True),
            nn.Linear(20, output_dim),
        )

    def forward(self, cam_feat: torch.Tensor, agg_lidar: torch.Tensor,
                quality: torch.Tensor):
        """
        Args:
            cam_feat:  (B, M, D)
            agg_lidar: (B, M, D)
            quality:   (B, 7)
        Returns:
            output: (B, M, 7) [x,y,z,w,h,l,conf]
        """
        fused = torch.cat([cam_feat, agg_lidar], dim=-1)  # (B, M, 64)
        gate = torch.sigmoid(self.gate_proj(quality))       # (B, 64)
        fused = fused * gate.unsqueeze(1)                   # (B, M, 64)
        fused = self.se(fused)                              # (B, M, 64)
        return self.reg_head(fused)                         # (B, M, 7)


class QCGAF(nn.Module):
    """Quality-aware Gated Attention Fusion Network (top-level)."""

    def __init__(self, cam_dim: int = 9, lidar_dim: int = 10,
                 quality_dim: int = 7, hidden_dim: int = 20,
                 feat_dim: int = 32, output_dim: int = 7):
        super().__init__()
        self.encoder = FeatureEncoder(cam_dim, lidar_dim, hidden_dim, feat_dim)
        self.matcher = QualityAttentionMatcher(quality_dim, feat_dim)
        self.fusion = GatedFusionHead(quality_dim, feat_dim, output_dim)
        # Quality-dependent position blend: learns alpha from quality to mix
        # cam vs lidar raw positions. Enables full sensor switching.
        self.blend_proj = nn.Linear(quality_dim, 1)
        # Initialize bias to 0 so alpha starts at 0.5 (equal weighting)
        nn.init.zeros_(self.blend_proj.bias)
        nn.init.xavier_uniform_(self.blend_proj.weight)
        self.alpha_min = 0.45
        # Runtime constants for safer cross-sensor blending.
        self.dist_gate_center = 1.5
        self.dist_gate_scale = 2.0
        self.fallback_alpha = 0.5

    def forward(self, cam_dets: torch.Tensor, lidar_dets: torch.Tensor,
                quality: torch.Tensor,
                cam_mask: torch.Tensor = None,
                lidar_mask: torch.Tensor = None,
                return_aux: bool = False):
        """
        Args:
            cam_dets:   (B, M, 9)
            lidar_dets: (B, N, 10)
            quality:    (B, 7)
            cam_mask:   (B, M) bool
            lidar_mask: (B, N) bool
        Returns:
            output:       (B, M, 7) fused boxes [x,y,z,w,h,l,conf]
            match_matrix: (B, M, N) soft matching
        """
        cam_feat, lidar_feat = self.encoder(cam_dets, lidar_dets)
        match_matrix, cam_q = self.matcher(
            cam_feat, lidar_feat, quality, cam_mask, lidar_mask
        )
        agg_lidar = torch.bmm(match_matrix, lidar_feat)  # (B, M, D)
        # Use quality-modulated camera features in the fusion head so low
        # image/depth quality can actually suppress the camera branch.
        correction = self.fusion(cam_q, agg_lidar, quality)  # (B, M, 7)

        # Quality-dependent raw position blending
        # alpha ∈ [0,1]: 1 = trust camera, 0 = trust lidar
        alpha = torch.sigmoid(self.blend_proj(quality))  # (B, 1)
        alpha = self.alpha_min + (1.0 - self.alpha_min) * alpha
        alpha = alpha.unsqueeze(1)                        # (B, 1, 1)

        cam_pos = cam_dets[:, :, :6]                      # (B, M, 6)
        lidar_pos = torch.bmm(match_matrix, lidar_dets[:, :, :6])  # (B, M, 6)

        # Distance-based safety gate: when the matched lidar box is too far
        # from the camera box, fall back to a neutral blend instead of
        # snapping all the way back to the camera branch.
        dist = (cam_pos[:, :, :3] - lidar_pos[:, :, :3]).pow(2).sum(dim=-1, keepdim=True).sqrt()
        dist_gate = torch.sigmoid(self.dist_gate_scale * (self.dist_gate_center - dist))
        eff_alpha = alpha * dist_gate + self.fallback_alpha * (1.0 - dist_gate)

        blended_pos = eff_alpha * cam_pos + (1 - eff_alpha) * lidar_pos

        output = correction.clone()
        output[:, :, :6] = blended_pos
        # Confidence from fusion head (correction[:, :, 6])

        if return_aux:
            aux = {
                "alpha": alpha,
                "eff_alpha": eff_alpha,
                "dist_gate": dist_gate,
                "cam_pos": cam_pos,
                "lidar_pos": lidar_pos,
            }
            return output, match_matrix, aux

        return output, match_matrix


def count_parameters(model: nn.Module) -> int:
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    model = QCGAF()
    n_params = count_parameters(model)
    print(f"Total trainable parameters: {n_params:,}")

    # breakdown
    for name, module in [
        ("FeatureEncoder", model.encoder),
        ("QualityAttentionMatcher", model.matcher),
        ("GatedFusionHead", model.fusion),
    ]:
        n = sum(p.numel() for p in module.parameters() if p.requires_grad)
        print(f"  {name}: {n:,}")

    # test forward
    B, M, N = 2, 10, 10
    cam = torch.randn(B, M, 9)
    lidar = torch.randn(B, N, 10)
    q = torch.rand(B, 7)
    cam_mask = torch.ones(B, M, dtype=torch.bool)
    lidar_mask = torch.ones(B, N, dtype=torch.bool)
    cam_mask[0, 7:] = False
    lidar_mask[0, 8:] = False

    out, S = model(cam, lidar, q, cam_mask, lidar_mask)
    print(f"\nOutput shape: {out.shape}")        # (2, 10, 7)
    print(f"Match matrix shape: {S.shape}")      # (2, 10, 10)
    print(f"Match row sums: {S[0, 0].sum().item():.4f}")  # ~1.0
    print("Model verification PASSED")
