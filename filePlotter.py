import matplotlib.pyplot as plt
from utilities import FileReader
import argparse

# Define markers and colors for differentiation
MARKERS = ["o", "s", "D", "^", "v", "P", "*", "x"]
COLORS = ["b", "g", "r", "c", "m", "y", "k", "orange"]


def plot_sensor_data(filename):
    """Plots sensor data from the given CSV file with appropriate labels and styles."""

    headers, values = FileReader(filename).read_file()

    if not values:
        print(f"Warning: No data found in {filename}")
        return

    time_list = []
    first_stamp = values[0][-1]

    # Convert timestamps to relative time
    for val in values:
        time_list.append(val[-1] - first_stamp)

    plt.figure(figsize=(10, 6))  # Set figure size

    # Loop through each column except the last one (timestamp)
    for i in range(len(headers) - 1):
        plt.plot(
            time_list,
            [lin[i] for lin in values],
            label=headers[i],
            marker=MARKERS[i % len(MARKERS)],
            color=COLORS[i % len(COLORS)],
            linestyle="-",
            markersize=4,  # Reduce marker size
        )

    # Extract motion type from filename
    motion_type = filename.split("_")[-1].split(".")[
        0
    ]

    plt.xlabel("Time (nanoseconds)")
    plt.ylabel("Sensor Values")
    plt.title(f"Sensor Data Plot - {motion_type.capitalize()} Motion")
    plt.legend()
    plt.grid(True, linestyle="--", alpha=0.6)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Plot logged sensor data from CSV files."
    )
    parser.add_argument(
        "--files", nargs="+", required=True, help="List of CSV files to process"
    )

    args = parser.parse_args()

    print("Plotting the following files:", args.files)

    for filename in args.files:
        plot_sensor_data(filename)
