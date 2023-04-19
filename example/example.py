"""
This is the example file that shows how program of
finding road bounds work.
"""

from pathlib import Path
from detection.detection import find_road_bounds


HOME_PATH = Path(__file__).parents[1]


if __name__ == '__main__':
    data_path = str(HOME_PATH / 'example' / 'data.las')
    save_path = str(HOME_PATH / 'example' / 'road_bound.las')
    find_road_bounds(data_path, save_path)
