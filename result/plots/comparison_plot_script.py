#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os

def create_comparison_plots(csv_file, output_dir):
    try:
        data = pd.read_csv(csv_file)

        # 4 子图
        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(18, 12))

        def draw(ax, key_meas, key_pred, key_filt, key_ahead, title, ylabel):
            ax.plot(data['time'], data[key_meas], 'r--', linewidth=1, label='Measured', alpha=0.7)
            ax.plot(data['time'], data[key_pred], 'g-.', linewidth=1, label='Predicted(t+dt)')
            ax.plot(data['time'], data[key_filt], 'b-',  linewidth=2, label='Filtered')
            ax.plot(data['time'], data[key_ahead], 'm:', linewidth=2, label='Ahead')
            ax.set_xlabel('Time (s)')
            ax.set_ylabel(ylabel)
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_title(title)

        draw(ax1, 'meas_x', 'predicted_x', 'filtered_x', 'ahead_x', 'X Position Comparison', 'X Position')
        draw(ax2, 'meas_y', 'predicted_y', 'filtered_y', 'ahead_y', 'Y Position Comparison', 'Y Position')
        draw(ax3, 'meas_z', 'predicted_z', 'filtered_z', 'ahead_z', 'Z Position Comparison', 'Z Position')
        draw(ax4, 'meas_yaw', 'predicted_yaw', 'filtered_yaw', 'ahead_yaw', 'Yaw Angle Comparison', 'Yaw Angle (rad)')

        plt.tight_layout()
        output_path = os.path.join(output_dir, 'ekf_comparison_results.png')
        plt.savefig(output_path, dpi=300, bbox_inches='tight')
        plt.close()

        print(f"Combined comparison plot saved to {output_path}")

    except Exception as e:
        print(f"Error creating comparison plots: {e}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python plot_script.py <csv_file> <output_dir>")
        sys.exit(1)

    create_comparison_plots(sys.argv[1], sys.argv[2])
