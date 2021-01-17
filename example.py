"""
This is the example file that shows how program of
finding road bounds work.
"""
from detection import find_road_bounds


if __name__ == '__main__':
    out_path = find_road_bounds()
    print(out_path)
