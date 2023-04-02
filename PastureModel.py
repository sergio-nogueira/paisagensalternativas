import sys
import gdal, ogr, osr
import numpy as np
import rasterio


def getRasterData(shapefile, feature, rasterfile, rasterfile2):

    raster = gdal.Open(rasterfile)
    raster2 = gdal.Open(rasterfile2)

    shpDs = ogr.Open(shapefile)
    shpLayer = shpDs.GetLayer()
    
    geometry = feature.GetGeometryRef()

    memDriver = ogr.GetDriverByName("Memory")
    tempDs = memDriver.CreateDataSource("tempDS")
    srs = shpLayer.GetSpatialRef()
    tempLayer = tempDs.CreateLayer("tempLayer", srs, geometry.GetGeometryType())

    tempFeature = ogr.Feature(shpLayer.GetLayerDefn())
    tempFeature.SetGeometry(geometry)
    tempLayer.CreateFeature(tempFeature)

    # Get extent of feat
    geom = feature.GetGeometryRef()
    if (geom.GetGeometryName() == 'MULTIPOLYGON'):
        count = 0
        pointsX = []
        pointsY = []
        for polygon in geom:
            geomInner = geom.GetGeometryRef(count)
            ring = geomInner.GetGeometryRef(0)
            numpoints = ring.GetPointCount()
            for p in range(numpoints):
                    lon, lat, z = ring.GetPoint(p)
                    pointsX.append(lon)
                    pointsY.append(lat)
            count += 1
    elif (geom.GetGeometryName() == 'POLYGON'):
        ring = geom.GetGeometryRef(0)
        numpoints = ring.GetPointCount()
        pointsX = []
        pointsY = []
        for p in range(numpoints):
                lon, lat, z = ring.GetPoint(p)
                pointsX.append(lon)
                pointsY.append(lat)

    else:
        sys.exit("ERROR: Geometry needs to be either Polygon or Multipolygon")

    transform = raster.GetGeoTransform()
    xOrigin = transform[0]
    yOrigin = transform[3]
    pixelWidth = transform[1]
    pixelHeight = transform[5]

    xmin = min(pointsX)
    xmax = max(pointsX)
    ymin = min(pointsY)
    ymax = max(pointsY)

    xmin = xmin - (xmin % pixelWidth)
    xmax = xmax - (xmax % pixelWidth)
    ymin = ymin - (ymin % pixelWidth)
    ymax = ymax - (ymax % pixelWidth)

    xoff = int((xmin - xOrigin)/pixelWidth)
    yoff = int((yOrigin - ymax)/pixelWidth)
    xcount = int((xmax - xmin)/pixelWidth)+1
    ycount = int((ymax - ymin)/pixelWidth)+1

    tempRaster = gdal.GetDriverByName('MEM').Create('', xcount, ycount, gdal.GDT_Byte)
    #tempRaster = gdal.GetDriverByName('GTiff').Create('teste.tif', xcount, ycount, gdal.GDT_Byte)
    tempRaster.SetGeoTransform((
        xmin, pixelWidth, 0,
        ymax, 0, pixelHeight,
    ))

    tempRasterSrs = osr.SpatialReference()
    tempRasterSrs.ImportFromWkt(raster.GetProjectionRef())
    tempRaster.SetProjection(tempRasterSrs.ExportToWkt())

    gdal.RasterizeLayer(tempRaster, [1], tempLayer, burn_values=[1], options = []) #, options = ["ALL_TOUCHED=TRUE", "BURN_VALUE_FROM"])

    banddataraster = raster.GetRasterBand(1)
    dataraster = banddataraster.ReadAsArray(xoff, yoff, xcount, ycount).astype(np.byte)

    banddataraster2 = raster2.GetRasterBand(1)
    dataraster2 = banddataraster2.ReadAsArray(xoff, yoff, xcount, ycount)

    bandmask = tempRaster.GetRasterBand(1)
    datamask = bandmask.ReadAsArray(0, 0, xcount, ycount).astype(np.byte)

    dataraster[np.logical_not(datamask==1)] = 0
    dataraster2[np.logical_not(datamask==1)] = 0

    return np.copy(dataraster), np.copy(dataraster2), tempRaster

def data2Tiff(data, filename, tempRaster):
    drv = gdal.GetDriverByName("GTiff")
    width, height = data.shape
    ds = drv.Create(filename, height, width, 1, gdal.GDT_Float32)
    ds.SetGeoTransform(tempRaster.GetGeoTransform())

    resultSrs = osr.SpatialReference()
    resultSrs.ImportFromWkt(tempRaster.GetProjectionRef())
    ds.SetProjection(resultSrs.ExportToWkt())
    

    ds.GetRasterBand(1).WriteArray(data)

def calcPastureArea(pasture):
    uniques, counts = np.unique(pasture, return_counts=True)
    
    values = {}
    for i in range(0, len(uniques)):
        values[str(uniques[i])] = counts[i]

    pastureAreaNoCC = 0
    if '1' in values:
        pastureAreaNoCC = values['1']
    
    pastureAreaCC = 0
    if '2' in values:
        pastureAreaCC = values['2']

    pastureArea = pastureAreaNoCC + pastureAreaCC

    return (pastureArea*0.09), (pastureAreaNoCC*0.09), (pastureAreaCC*0.09)

def calcThresholdIpcpa(ipcpa, pasture, expectedArea, pastureTypes, bestValues = False):
    
    pastureIpcpa = np.copy(ipcpa)
    pastureIpcpa[np.logical_not(np.isin(pasture, pastureTypes ))] = 0

    uniques, counts = np.unique(pastureIpcpa, return_counts=True)
    counts[0] = 0

    if bestValues:
        uniques = np.flip(uniques,0)
        counts = np.flip(counts,0)

    areaIpcpa = 0
    thresholdIpcpa = 0
    for i, c in enumerate(counts):
        areaIpcpa = areaIpcpa + c
        thresholdIpcpa = uniques[i]
        if (areaIpcpa * 0.09) >= expectedArea:
            break

    areaIpcpa = areaIpcpa * 0.09
    if bestValues:
        pastureIpcpa = np.where(np.logical_and(pastureIpcpa > 0, pastureIpcpa >= thresholdIpcpa), 1, 0)
    else:
        pastureIpcpa = np.where(np.logical_and(pastureIpcpa > 0, pastureIpcpa <= thresholdIpcpa), 1, 0)

    return pastureIpcpa, areaIpcpa

shapefile = sys.argv[1]
rasterfile = sys.argv[2]
rasterfile2 = sys.argv[3]

pastureAreaFactor = 0.5

shpDs = ogr.Open(shapefile)
shpLayer = shpDs.GetLayer()
featureCount = shpLayer.GetFeatureCount()
#shpLayer.SetAttributeFilter("CD_GEOCMU = '5204250'")
#shpLayer.SetAttributeFilter("CD_GEOCMU = '5102637'")
#shpLayer.SetAttributeFilter("CD_GEOCMU = '5107040'")

cerradoRestauration = 0
cerradoPotentialCrop = 0
cerradoNewPasture = 0

count = 1
for feature in shpLayer:
    
    pasture, ipcpa, tempRaster = getRasterData(shapefile, feature, rasterfile, rasterfile2)
    pastureArea, pastureAreaNoCC, pastureAreaPC = calcPastureArea(pasture)
    ipcpa = (ipcpa*10000).astype('int16')

    if (pastureArea <= 0):
        continue
    
    geocode = feature.GetField('CD_MUN')
    potencialRestArea = feature.GetField('Restaur_ha')
        
    minPastureArea = pastureArea * pastureAreaFactor
    pastureSparing = (pastureArea - minPastureArea)
    
    print("## Processing " + geocode + " [" + str(count) + "/" + str(featureCount) + "]")

    print("   Total pasture: " + str(pastureArea) + " ha")
    print("   Pasture (No-PC): " + str(pastureAreaNoCC) + " ha")
    print("   Potential crop: " + str(pastureAreaPC) + " ha")
    print("   Potential Restauration: " + str(potencialRestArea) + " ha")
    print("   Min pasture: " + str(minPastureArea) + " ha")
    print("   Pasture sparing: " + str(pastureSparing) + " ha")

    if (potencialRestArea > 0):

        realRestArea = min(pastureSparing, potencialRestArea)

        restPasture1, realRestArea1 = calcThresholdIpcpa(ipcpa, pasture, realRestArea, 1, bestValues=False)

        if (realRestArea - realRestArea1) > 0:
            restPasture2, realRestArea2 = calcThresholdIpcpa(ipcpa, pasture, (realRestArea - realRestArea1), 2, bestValues=False)
            restPasture = np.logical_or(restPasture1,restPasture2)

            realRestArea = realRestArea1 + realRestArea2
        else:
            restPasture = restPasture1

        pasture[restPasture == 1] = 0
        pastureSparing = pastureSparing - realRestArea
        cerradoRestauration = cerradoRestauration + realRestArea

        data2Tiff(restPasture, "D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/RESTAURATION_MUN/" + str(geocode) + ".tif", tempRaster)
        #saida_rest = np.int8()
        #restPasture.write("D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/RESTAURATION_MUN/" + str(geocode) + ".tif", 1)
        #restPasture.close()
  
        print("    Restauration: " + str(realRestArea) + " ha")

    if (pastureSparing > 0  and pastureAreaPC > 0):
        
        realPastureAreaPC = min(pastureSparing, pastureAreaPC)
        realPasturePC, realPastureAreaPC2 = calcThresholdIpcpa(ipcpa, pasture, realPastureAreaPC, 2, bestValues=True)
        
        pasture[realPasturePC == 1] = 0
        pastureSparing = pastureSparing - realPastureAreaPC
        cerradoPotentialCrop = cerradoPotentialCrop + realPastureAreaPC

        data2Tiff(realPasturePC, "D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/POTENTIAL_CROP_MUN/" + str(geocode) + ".tif", tempRaster)
        #realPasturePC.write('D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/POTENTIAL_CROP_MUN/' + str(geocode) + '.tif', 1)
        #realPasturePC.close()

        print("    Potential crop: " + str(realPastureAreaPC) + " ha")

    newPastureArea = minPastureArea + pastureSparing
    cerradoNewPasture = cerradoNewPasture + newPastureArea

    data2Tiff(pasture, "D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/PASTURE_MUN/" + str(geocode) + ".tif", tempRaster)
    #pasture.write('D:/Tese/Cap3/Pasture_cerrado_model/OUTPUT/PASTURE_MUN/' + str(geocode) + '.tif', 1)
    #pasture.close()
    print("    New pasture area: " + str(newPastureArea) + " ha")
    count = count + 1

print("## Finish")
print("    Cerrado restauration:" + str(cerradoRestauration))
print("    Cerrado potential crop:" + str(cerradoPotentialCrop))
print("    Cerrado pasture area::" + str(cerradoNewPasture))