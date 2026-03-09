import numpy as np
from numpy.typing import NDArray
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

from pyekf.quaternions import (
    normalize_quaternion,
    angular_distance_degrees,
    quat_to_euler,
)


class ResultsCollector:
    def __init__(
        self,
        title: str = "Quaternion Angular Error",
        metadata: dict | None = None,
    ):
        self.title = title
        self.metadata = metadata or {}

        self.times: list[float] = []
        self.gt_quaternions: list[np.ndarray] = []
        self.est_quaternions: list[np.ndarray] = []

    def collect(
        self,
        time: float,
        gt_state: NDArray[np.float64],
        ekf_nominal_state: NDArray[np.float64]
    ):
        self.times.append(time)
        self.gt_quaternions.append(normalize_quaternion(gt_state))
        self.est_quaternions.append(normalize_quaternion(ekf_nominal_state))

    def _format_metadata_block(self) -> str:
        if not self.metadata:
            return ""

        lines = []
        for key, value in self.metadata.items():
            if isinstance(value, float):
                lines.append(f"{key}: {value:.6g}")
            else:
                lines.append(f"{key}: {value}")
        return " | ".join(lines)

    def save_and_show(
        self,
        output_dir: str = "test_plots",
        filename: str = "orientation_analysis.png",
    ):
        t = np.array(self.times)

        # 1. Calculate Angular Distance
        angular_errors_deg = np.array([
            angular_distance_degrees(q_true, q_est)
            for q_true, q_est in zip(self.gt_quaternions, self.est_quaternions)
        ])

        # 2. Calculate Euler Errors (Roll, Pitch, Yaw)
        gt_eulers = np.array([quat_to_euler(q).flatten() for q in self.gt_quaternions])
        est_eulers = np.array([quat_to_euler(q).flatten() for q in self.est_quaternions])
        
        euler_errors_rad = gt_eulers - est_eulers
        euler_errors_deg = np.degrees((euler_errors_rad + np.pi) % (2 * np.pi) - np.pi)

        fig, axes = plt.subplots(4, 1, figsize=(12, 14), sharex=True)
        fig.suptitle(self.title, fontsize=16, fontweight="bold")

        labels = ["Total Angular Error (deg)", "Roll Error (deg)", "Pitch Error (deg)", "Yaw Error (deg)"]
        colors = ["black", "tab:red", "tab:green", "tab:blue"]
        
        # Combine data for easier iteration
        data_to_plot = [angular_errors_deg] + [euler_errors_deg[:, i] for i in range(3)]

        for i, ax in enumerate(axes):
            ax.plot(t, data_to_plot[i], linewidth=1.5, color=colors[i], label=labels[i])
            
            if i > 0: # RPY plots
                ax.axhline(0, color='black', linewidth=0.8, alpha=0.5)
            else: # Total error plot
                mean_err = float(np.mean(angular_errors_deg))
                ax.axhline(mean_err, linestyle="--", color='gray', label=f"Mean = {mean_err:.4f}")

            # --- FIX: Remove scientific notation and offsets ---
            ax.yaxis.set_major_formatter(matplotlib.ticker.ScalarFormatter(useOffset=False))
            ax.ticklabel_format(style='plain', axis='y') # Force "plain" decimal format
            
            ax.set_ylabel(labels[i])
            ax.grid(True, alpha=0.3)
            ax.legend(loc="upper right")

        axes[-1].set_xlabel("Time (s)")
        
        metadata_text = self._format_metadata_block()
        if metadata_text:
            fig.text(0.5, 0.01, metadata_text, ha="center", va="bottom", fontsize=9, family="monospace")

        plt.tight_layout(rect=[0, 0.03, 1, 0.96])
        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=200)
        print(f"\n>>> Orientation analysis saved to: {filepath}")
        plt.close(fig)
