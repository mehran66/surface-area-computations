'''----------------------------------------------------------------------------------
File Name      : numpy2Raster.py
Author         : Mehran Ghandehari
Organization   : University of Colorado at Boulder
Created        : Nov. 10th, 2017
Python Version : 2.7

-- Description --
This module converts a numpy array to a geotiff
----------------------------------------------------------------------------------'''
import numpy as np
from osgeo import gdal

def convert(array, minX, minY, maxX, maxY, xres, sourceProj, output):
    '''
    array: is the input numpy array that is going to convert to a geotiff
    lat: the corresponding latitides of the array matrix
    long the corresponding longitues of the array matrix
    output: where the geotiff saves
    '''
    # Array properties
    nrows, ncols = np.shape(array)  # Array shape
    geotransform=(minX,xres,0,maxY,0, -xres)
    # That's (top left x, w-e pixel resolution, rotation (0 if North is up),
    #         top left y, rotation (0 if North is up), n-s pixel resolution)

    output_raster = gdal.GetDriverByName('GTiff').Create(output + '.tif',ncols, nrows, 1 ,gdal.GDT_Float32)  # Open the file
    output_raster.SetGeoTransform(geotransform)  # Specify its coordinates
    output_raster.SetProjection(sourceProj)   # Exports the coordinate system to the file

    output_raster.GetRasterBand(1).WriteArray(array)   # Writes my array to the raster