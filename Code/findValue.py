# Find vale of a point in an array
import numpy as np

def find(lidar, x, y):

    #  lidarProperties = [arr_lidar, xres_lidar, yres_lidar,minX_lidar, maxX_lidar]
    array = lidar[0]
    xres_lidar = lidar[1]
    yres_lidar = lidar[2]
    minX_lidar = lidar[3]
    maxY_lidar = lidar[4]

    # Divide the difference between the x value of the point and origin,
    # and divide this by the resolution to get the raster index
    x_index = int((x - minX_lidar) / xres_lidar)

    # and the same as the y
    y_index = int((y - maxY_lidar) / yres_lidar)


    # Sample with the indexs, not that y_index should be first as the index is
    # [rows, columns] in a 2d grid in python

    if x_index >  np.size(array, 1)-1 or x_index< 0:
        return -1
    else:
        pixel_val = array[y_index, x_index]
        return pixel_val





# import rasterio
# with rasterio.open(r'E:\Surface_adjusted\ICC17\Code\Data\NC_cub\dem10m') as src:
#     for val in src.sample([(412786.732, 3969438.426)]):
#         print(val)