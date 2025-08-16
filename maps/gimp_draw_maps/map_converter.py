from PIL import Image
import numpy as np
import json

resolution = 0.05
image_path = './map_gimp_100_x_100_yellow.png'

position_x, position_y = None, None

def prepare_map():
    img = Image.open(image_path).convert('RGB')
    width, height = img.size
    np_img = np.array(img)

    occ_data = []

    for y in range(height):
        for x in range(width):
            r, g, b = np_img[y, x]

            if (r, g, b) == (0, 0, 0):
                occ_data.append(100)  # Occupied
            elif (r, g, b) == (255, 255, 255):
                occ_data.append(0)    # Free
            elif (r, g, b) == (255, 255, 0):
                occ_data.append(-1)   # Unknown
            elif (r, g, b) == (255, 0, 0):
                occ_data.append(0)    # Robot position = free
                position_x = x * resolution
                position_y = y * resolution
            else:
                occ_data.append(-1)   # Default unknown

    # Convert to JSON-like dict
    _json = {
    'odom': {
        'header': {
            'stamp': {
                'sec': 0,
                'nanosec': 0
            },
            'frame_id': 'odom'
        },
        'pose': {
            'pose': {
                'position': {
                    'x': position_x,
                    'y': position_y,
                    'z': 0.0
                },
                'orientation': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                    'w': 1.0
                }
            }
        }
    },
    'map': {
        'header': {
            'stamp': {
                'sec': 0,
                'nanosec': 0
            },
            'frame_id': 'map'
        },
        'info': {
            'map_load_time': {
                'sec': 0,
                'nanosec': 0
            },
            'resolution': resolution,
            'width': width,
            'height': height,
            'origin': {
                'position': {
                    'x': 0,
                    'y': 0,
                    'z': 0
                },
                'orientation': {
                    'x': 0,
                    'y': 0,
                    'z': 0,
                    'w': 1.0
                }
            }
        },
        'data': list(occ_data),
        }
    }

    # Print JSON
    print(json.dumps(_json,indent=4))

def main():
    prepare_map()

if __name__ == '__main__':
    main()
