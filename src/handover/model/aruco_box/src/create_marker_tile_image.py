import click
import cv2
from cv2 import aruco
import numpy as np
import yaml
import os

class MarkerFactory:

    @staticmethod
    def create_marker(size, id, margin):
        aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

        img = 255 * np.ones((size, size, 3), dtype=np.uint8)

        if id == 0:
            img[:] = (1, 200, 255)
        if id == 4:
            img[:] = (119, 80, 64)
        if id == 2:
            img[:] = (105, 186, 127)
        if id == 5:
            img[:] = (75, 75, 255)

        img[7:-7, 7:-7] = (255, 255, 255)

        # white background
        img_marker = cv2.cvtColor(aruco.drawMarker(aruco_dict, id, size - 2 * margin),cv2.COLOR_GRAY2RGB)

        img_marker = cv2.flip(img_marker, 1)

        # add marker centered
        img[margin:-margin, margin:-margin] = img_marker

        return img


class TileMap:
    _map: np.ndarray

    def __init__(self, tile_size):
        self._map = 255 * np.ones((4, 3, tile_size, tile_size, 3), dtype=np.uint8)

    def set_tile(self, pos: tuple, img: np.ndarray):
        assert np.all(self._map[pos[0], pos[1]].shape == img.shape)
        self._map[pos[0], pos[1]] = img

    def get_map_image(self):
        """ Merges the tile map into a single image """

        img = np.concatenate(self._map, axis=-2)
        img = np.concatenate(img, axis=-3)

        img = np.transpose(img, (1, 0, 2))

        return img


@click.command()
@click.argument("path", type=click.Path(exists=True))
@click.option("--tile_size", type=int, default=100)
def main(path, tile_size):
    margin = int(0.2 * tile_size)

    marker_factory = MarkerFactory()
    tile_map = TileMap(tile_size)

    order = ['left', 'botton', 'front', 'top' , 'back', 'right']

    ids = []

    marker_id = 0
    for i in range(4):
        for j in range(3):
            if i != 1 and (j==0  or j == 2):
                continue

            marker_img = marker_factory.create_marker(tile_size, marker_id, margin)
            tile_map.set_tile((i, j), marker_img)
            ids.append(marker_id)

            marker_id += 1

    tile_img = tile_map.get_map_image()

    tile_img_square = np.zeros((tile_size * 4, tile_size*4, 3))
    tile_img_square[:, (tile_size//2):(-tile_size//2)] = tile_img

    cv2.imwrite(os.path.join(path, "marker_tile.png"), tile_img)
    cv2.imwrite(os.path.join(path, "marker_tiles_square.png"), tile_img_square)

    marker_config = dict(zip(order, ids))

    config = dict()
    config["aruco_dict"] = "4X4_50"
    config["markers"] = marker_config

    with open(os.path.join(path, "marker_info.yml"), "w") as yml_file:
        yaml.dump(config, yml_file)


if __name__ == '__main__':
    main()