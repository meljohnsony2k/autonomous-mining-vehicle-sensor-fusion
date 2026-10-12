import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Function to apply low-pass Butterworth filter
def butter_lowpass_filter(data, cutoff, fs, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

# Function to parse GNSS log data, handling both "Raw" and "Fix" types
def parse_gnss_data(file_path):
    raw_entries = []
    fix_entries = []

    with open(file_path, 'r') as file:
        for line in file:
            # Handle "Raw" data lines
            if line.startswith('Raw,'):
                line_data = line.strip().split(',')
                if len(line_data) > 28:  # Ensure enough fields are present
                    entry = {
                        'utcTimeMillis': int(line_data[1]),
                        'Cn0DbHz': float(line_data[16]) if line_data[16] else None,
                        'AccumulatedDeltaRangeUncertaintyMeters': float(line_data[21]) if line_data[21] else None,
                        'ConstellationType': int(line_data[28]) if line_data[28] else None
                    }
                    raw_entries.append(entry)
            
            # Handle "Fix" data lines
            elif line.startswith('Fix,'):
                line_data = line.strip().split(',')
                try:
                    entry = {
                        'utcTimeMillis': int(line_data[8]),  # UnixTimeMillis from the Fix data
                        'AccuracyMeters': float(line_data[6]) if line_data[6] else None  # Extracting AccuracyMeters
                    }
                    fix_entries.append(entry)
                except (ValueError, IndexError):
                    print(f"Skipping invalid Fix line: {line.strip()}")

    # Create DataFrames from both sets of entries
    raw_df = pd.DataFrame(raw_entries)
    fix_df = pd.DataFrame(fix_entries)

    # Combine the data for comprehensive analysis
    combined_df = pd.concat([raw_df, fix_df], ignore_index=True)

    return combined_df

# Load and parse the data
file_path = 'gnss_log_2024_09_01_13_16_13.txt'
gnss_data = parse_gnss_data(file_path)

# Filter data for each constellation type and apply the low-pass filter
cutoff = 0.02
fs = 1.0
constellation_types = {1: 'GPS', 3: 'GLONASS', 5: 'Beidou', 6: 'Galileo'}
filtered_data = {}

for const_type, name in constellation_types.items():
    data = gnss_data[gnss_data['ConstellationType'] == const_type]
    if not data.empty:
        data = data.copy()  # Avoid SettingWithCopyWarning
        data['FilteredCn0DbHz'] = butter_lowpass_filter(data['Cn0DbHz'].dropna(), cutoff, fs)
        data['FilteredAccumulatedDeltaRangeUncertaintyMeters'] = butter_lowpass_filter(data['AccumulatedDeltaRangeUncertaintyMeters'].dropna(), cutoff, fs)
        filtered_data[name] = data

# Handle the AccuracyMeters filtering separately
if 'AccuracyMeters' in gnss_data.columns:
    gnss_data['FilteredAccuracyMeters'] = gnss_data['AccuracyMeters'].apply(lambda x: x if x <= 500 else None)
else:
    print("AccuracyMeters column is missing from the data.")

# Plotting setup with three rows, raw on the left, processed on the right
fig, axs = plt.subplots(3, 2, figsize=(20, 15), sharex=True)
colors = {'GPS': '#CCCC00', 'GLONASS': 'purple', 'Beidou': 'red', 'Galileo': 'darkgreen'}

# Define plot titles
plot_titles = [
    ("Raw Signal Quality (Cn0DbHz)", "Filtered Signal Quality (Cn0DbHz)"),
    ("Raw Accumulated Delta Range Uncertainty (Meters)", "Filtered Accumulated Delta Range Uncertainty (Meters)"),
    ("Raw Accuracy Meters", "Filtered Accuracy Meters (values <= 500 Meters)")
]

# Define the corresponding metrics
metrics = [
    ('Cn0DbHz', 'FilteredCn0DbHz'),
    ('AccumulatedDeltaRangeUncertaintyMeters', 'FilteredAccumulatedDeltaRangeUncertaintyMeters'),
    ('AccuracyMeters', 'FilteredAccuracyMeters')
]

# Plotting each metric with checks
for row, (metric_pair, title_pair) in enumerate(zip(metrics, plot_titles)):
    for col, (metric, title) in enumerate(zip(metric_pair, title_pair)):
        ax = axs[row, col]
        plotted = False  # Flag to check if any data is plotted
        for name, data in filtered_data.items():
            if metric in data.columns and not data[metric].isnull().all():
                ax.plot(pd.to_datetime(data['utcTimeMillis'], unit='ms'), data[metric], label=name, linestyle='-', color=colors[name])
                plotted = True
        # Only include 'Fix Data' for Accuracy Meters plots
        if metric in ['AccuracyMeters', 'FilteredAccuracyMeters'] and metric in gnss_data.columns and not gnss_data[metric].isnull().all():
            ax.plot(pd.to_datetime(gnss_data['utcTimeMillis'], unit='ms'), gnss_data[metric], label='Fix Data', linestyle='-', color='blue')
            plotted = True
        if plotted:
            ax.set_title(title)
            ax.set_ylabel('Meters' if 'Accuracy' in metric or 'DeltaRange' in metric else 'Cn0DbHz')
            ax.legend()
            ax.grid(True)

plt.xticks(rotation=45)
plt.tight_layout()
plt.show()
