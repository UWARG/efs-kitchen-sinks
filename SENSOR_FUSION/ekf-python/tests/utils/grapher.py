import numpy as np
from numpy.typing import NDArray
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

from pyekf.quaternions import (
    normalize_quaternion,
    angular_distance_degrees,
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
        filename: str = "quat_angular_error.png",
    ):
        t = np.array(self.times)

        angular_errors_deg = np.array([
            angular_distance_degrees(q_true, q_est)
            for q_true, q_est in zip(self.gt_quaternions, self.est_quaternions)
        ])

        mean_error = float(np.mean(angular_errors_deg))

        fig, ax = plt.subplots(figsize=(12, 6))
        fig.suptitle(self.title, fontsize=16, fontweight="bold")

        ax.plot(
            t,
            angular_errors_deg,
            linewidth=1.8,
            label="Angular Error (deg)",
        )

        ax.axhline(
            mean_error,
            linestyle="--",
            label=f"Mean Error = {mean_error:.4f} deg",
        )

        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Smallest Orientation Difference (deg)")
        ax.grid(True, alpha=0.3)
        ax.legend()

        upper_bound = max(np.max(angular_errors_deg) * 1.2, 0.01)
        ax.set_ylim(0.0, upper_bound)

        metadata_text = self._format_metadata_block()
        if metadata_text:
            fig.text(
                0.5,
                0.01,
                metadata_text,
                ha="center",
                va="bottom",
                fontsize=9,
                family="monospace",
            )

        plt.tight_layout(rect=[0, 0.05, 1, 0.95])

        os.makedirs(output_dir, exist_ok=True)
        filepath = os.path.join(output_dir, filename)
        plt.savefig(filepath, dpi=200)
        print(f"\n>>> Angular error analysis saved to: {filepath}")
        plt.close(fig)
