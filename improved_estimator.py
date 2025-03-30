import json
import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import label

def load_map(filepath):
    with open(filepath, 'r') as file:
        return json.load(file)

def process_map(json_data):
    width = json_data['info']['width']
    height = json_data['info']['height']
    data = np.array(json_data['data']).reshape((height, width))
    
    # Treat map boundaries as occupied
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100
    
    # Convert enclosed unknowns (-1) to occupied (100)
    occupied = data == 100
    unknown = data == -1
    labeled, num_features = label(unknown)
    for i in range(1, num_features + 1):
        region = labeled == i
        if np.any(occupied[region]):
            data[region] = 100
    
    # Close small gaps (<=5 pixels wide)
    for i in range(1, height - 1):
        for j in range(1, width - 1):
            if data[i, j] == -1:
                neighbors = data[i-1:i+2, j-1:j+2].flatten()
                if np.count_nonzero(neighbors == 100) >= 8:
                    data[i, j] = 100
    
    # Calculate occupancy percentage
    total_cells = width * height
    occupied_cells = np.count_nonzero(data == 100)
    occupied_percentage = (occupied_cells / total_cells) * 100
    print(f"Occupied Space: {occupied_percentage:.2f}%")
    
    return data, occupied_percentage

def visualize_map(data, percentage):
    plt.figure(figsize=(6, 6))
    plt.imshow(data, cmap='gray', origin='upper')
    plt.title(f"Map with {percentage:.2f}% Occupied Space")
    plt.axis('off')
    plt.show()

def main():
    filepath = 'maps/example_maps_v3/map_20250124_212336_852796.json'  # Adjust path as needed
    json_data = load_map(filepath)
    processed_data, occupied_percentage = process_map(json_data)
    visualize_map(processed_data, occupied_percentage)

if __name__ == '__main__':
    main()
