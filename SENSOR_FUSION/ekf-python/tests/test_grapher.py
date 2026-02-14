import numpy as np
import matplotlib
matplotlib.use('Agg') 
import matplotlib.pyplot as plt
import os

class ResultsCollector:
    def __init__(self, title="Quaternion Attitude Tracking"):
        self.title = title
        self.t, self.gt_q, self.est_q = [], [], []
        self.output_dir = "test_plots"
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)

    def collect(self, time, gt_state, ekf_nominal_state):
        self.t.append(time)
        self.gt_q.append(gt_state[3].flatten())
        self.est_q.append(ekf_nominal_state.quaternion_new.flatten())

    def save_and_show(self, filename="quat_analysis.png"):
        # Convert lists to numpy arrays for math
        t = np.array(self.t)
        gt_q = np.array(self.gt_q)
        est_q = np.array(self.est_q)
        
        # Calculate residuals (Error = Truth - Estimate)
        # Note: For quats, this is a linear approximation of error
        error_q = gt_q - est_q

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(self.title, fontsize=16, fontweight='bold')

        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'] # Consistent colors for w,x,y,z
        labels = ['w', 'x', 'y', 'z']

        # TOP PLOT: Absolute Tracking
        for i in range(4):
            ax1.plot(t, gt_q[:, i], '--', color=colors[i], alpha=0.4, label=f"GT {labels[i]}")
            ax1.plot(t, est_q[:, i], '-', color=colors[i], linewidth=1.5, label=f"Est {labels[i]}")
        
        ax1.set_title("Quaternion Components (Unitless)", loc='left', fontsize=12)
        ax1.set_ylabel("Amplitude")
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize='small')

        # BOTTOM PLOT: Residual Error (The "Truth" Plot)
        for i in range(4):
            ax2.plot(t, error_q[:, i], '-', color=colors[i], label=f"Err {labels[i]}")
        
        ax2.axhline(0, color='black', linewidth=1, alpha=0.7) # Zero error line
        ax2.set_title("Estimation Error (Truth - Estimate)", loc='left', fontsize=12)
        ax2.set_ylabel("Error Value")
        ax2.set_xlabel("Time (s)")
        ax2.grid(True, alpha=0.3, linestyle=':')
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5), fontsize='small')

        plt.tight_layout(rect=[0, 0.03, 0.88, 0.95])
        
        max_err = np.max(np.abs(error_q))
        dynamic_limit = max(max_err * 1.2, 0.005)
        ax2.set_ylim(-dynamic_limit, dynamic_limit)
        
        filepath = os.path.join(self.output_dir, filename)
        plt.savefig(filepath, dpi=200) # Higher DPI for clearer lines
        print(f"\n>>> High-utility analysis saved to: {filepath}")
        plt.close(fig)
