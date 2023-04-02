#!/bin/bash

rm -v OUTPUT/RESTAURATION_MUN/*.* OUTPUT/POTENTIAL_CROP_MUN/*.* OUTPUT/PASTURE_MUN/*.* OUTPUT/*.*

MUNICIPALITIES='INPUT/municipalities_albers.shp'
PASTURE_CROP_CONDITIONS='INPUT/pasture_albers.tif'
IPCPA='INPUT/prob_agricultura.tif'

python -u PastureModel.py $MUNICIPALITIES $PASTURE_CROP_CONDITIONS $IPCPA

gdal_merge.py -n 0 -o OUTPUT/pasture.tif -co COMPRESS=LZW -co TILED=YES -ot Byte OUTPUT/PASTURE_MUN/*.tif
gdal_merge.py -n 0 -o OUTPUT/potential_crop.tif -co COMPRESS=LZW -co TILED=YES -ot Byte OUTPUT/POTENTIAL_CROP_MUN/*.tif
gdal_merge.py -n 0 -o OUTPUT/restauration.tif -co COMPRESS=LZW -co TILED=YES -ot Byte OUTPUT/RESTAURATION_MUN/*.tif

gdaladdo --config USE_RRD YES -ro OUTPUT/pasture.tif 2 4 8
gdaladdo --config USE_RRD YES -ro OUTPUT/potential_crop.tif 2 4 8
gdaladdo --config USE_RRD YES -ro OUTPUT/restauration.tif 2 4 8