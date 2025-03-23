from json import load

DIRPATH = 'example_maps_v3'
MAP_001 = 'map_20250124_212336_852796.json'

val_free = 0
val_occupied = 100
val_unknown = -1

key_data = 'data'
key_resolution = 'resolution'
key_info = 'info'
key_width = 'width'
key_height = 'width'

def calculate_remaining_space():
    _data = dict()
    with open(DIRPATH + '/' + MAP_001, 'r+') as json_data:
        _data = load(json_data)
        json_data.close()
        
    _width = _data[key_info][key_width]
    _height = _data[key_info][key_height]
    _points = _data[key_data]
    
    
    _explored_num = sum(x == val_occupied or x == val_free for x in _points)
    print(_width, _height, _width*_height, _explored_num)
    _explored_percent = _explored_num / (_width * _height)
    print('explored %f not explored %f', _explored_percent, 1.0-_explored_percent)
    
    return

calculate_remaining_space()
    