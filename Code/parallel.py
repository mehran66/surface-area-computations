import surfaceArea  # Used to calculate surface area for each Pixel
import numpy as np


def  par(A):

    i, arr, X, Y, mth, xres, yres, lidar = A

    SA = []
    for j in np.arange(2, np.size(arr, 1) - 2):

        # Extract the values of a 5 by 5 moveing window
        rasterBlock_elev = arr[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the elevation of 25 neighbor pixels
        rasterBlock_x = X[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the x coordinate of 25 neighbor pixels
        rasterBlock_y = Y[0:6, j - 2:j + 3]  # this 5 by 5 matrix would contain the y coordinate of 25 neighbor pixels

        # Check for no data values that happend on any pixel
        if np.any(rasterBlock_elev < -3000000) or lidar[5] == -1:
            SA.append(np.nan)
        else:
            # Calculate surface area for different methods using the SurfaceArea module
            SA.append(surfaceArea.area(mth, rasterBlock_elev, rasterBlock_x, rasterBlock_y, xres, yres, lidar))

    return SA










