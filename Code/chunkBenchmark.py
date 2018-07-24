
def find(x, y, yres, lidarProperties):
    #  lidarProperties = [X_lidar, Y_lidar, arr_lidar, xres_lidar, yres_lidar,minX_lidar, minY_lidar, maxX_lidar, maxY_lidar]
    Y_lidar = lidarProperties[1]
    yres_lidar = lidarProperties[4]
    minY_lidar = lidarProperties[6]
    maxY_lidar = lidarProperties[8]

    # Divide the difference between the x value of the point and origin,
    # and divide this by the resolution to get the raster index
    if (y+(yres/2.0)) > maxY_lidar or (y-(yres/2.0)) < minY_lidar:
        y_index_min= -1
        y_index_max = -2
    else:
        y_index_min = int(((y+(yres/2.0)) - maxY_lidar) / yres_lidar)
        y_index_max = int(((y-(yres/2.0)) - maxY_lidar) / yres_lidar)


    return y_index_min, y_index_max, (Y_lidar[y_index_min,0]-(yres_lidar/2.0))
