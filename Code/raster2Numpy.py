# This module convert a raster to a numpy array and get the properties of the raster file using GDAL

# Import modules

import numpy as np
from osgeo import gdal

def convert(raster):
    '''
    param raster: the input raster file
    return: numpy array and the properties of the input raster
    '''

    geo = gdal.Open(raster)
    arr = geo.ReadAsArray()
    ulx, xres, xskew, uly, yskew, yres = geo.GetGeoTransform()
    lrx = ulx + (geo.RasterXSize * xres)
    lry = uly + (geo.RasterYSize * yres)
    minX, minY, maxX, maxY = ulx, lry, lrx, uly

    # Setup the source projection - you can also import from epsg, proj4...
    sourceProj = geo.GetProjectionRef()

    # generate X and Y matrices
    X = np.arange(minX + (xres / 2.0), maxX, xres)
    Y = np.arange(minY + abs(yres / 2.0), maxY, abs(yres))
    X, Y = np.meshgrid(X, Y)
    Y = Y[::-1]

    return X, Y, arr, xres, yres, minX, minY, maxX, maxY, sourceProj