#import arc license and set workspace
import arcpy
import math
from arcpy import env
from arcpy.sa import *
env.workspace = r'E:\Surface_adjusted\SurfaceArea\Code\python_V3\data2'
output = r"E:\Surface_adjusted\SurfaceArea\Code\python_V3\Results"
env.overwriteOutput = 'True'

# Check out the ArcGIS Spatial Analyst extension license
arcpy.CheckOutExtension("Spatial")


DEMs = ['dem10m', 'dem30m', 'dem100m', 'dem1000m']

for dem in DEMs:
    # Calculate surface area
    # Surface are is calculated for the whole DEM, because clipping DEM for each watershed is time-consuming
    slp = Slope(dem, "DEGREE") # Execute Slope
    slpRadian = slp * 0.0174533 # converting degree to radian
    cellSize = arcpy.GetRasterProperties_management(dem, "CELLSIZEX")
    surfaceArea = float(float(cellSize[0]) ** 2) / Cos(slpRadian) # calculating surface area
    surfaceArea.save(output + '\SA_' + 'slp' + str(int(float(cellSize[0]))) + '.tif')
    
    planar = float(float(cellSize[0]) ** 2) * Raster(dem) / Raster(dem)
    planar.save(output + '\SA_' + 'planar' + str(int(float(cellSize[0]))) + '.tif')