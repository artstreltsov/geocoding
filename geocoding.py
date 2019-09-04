# -*- coding: utf-8 -*-
"""
Created on Tue Aug 27 22:13:40 2019

@author: Artem Streltsov

This collection of functions trinagulates pixel coordinates
into geocoordinates

"""

import pandas as pd
import numpy as np
import os, glob
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.geometry.multipolygon import MultiPolygon
import scipy.interpolate
from osgeo import gdal
if 'GDAL_DATA' not in os.environ:
    os.environ['GDAL_DATA'] = os.path.join(os.getcwd(),'gdal-data')

import osgeo.osr as osr
import pyproj



def get_corners_from_image(image_path, epsg=4326):
    '''Use metadata of a georeferenced image to obtain its corner
    coordinates and shape.
    image_path is the path to the georeferenced image;
    epsg is the CRS to project corner coordinates to
    '''
    
    ds = gdal.Open(image_path)
    width = ds.RasterXSize
    height = ds.RasterYSize
    gt = ds.GetGeoTransform()
    minx = gt[0]
    miny = gt[3] + width*gt[4] + height*gt[5] 
    maxx = gt[0] + width*gt[1] + height*gt[2]
    maxy = gt[3] 
    
    x_sw,y_sw,x_nw,y_nw,x_ne,y_ne,x_se,y_se=minx,miny,minx,maxy,maxx,maxy,maxx,miny

    proj_wkt = ds.GetProjection()
    sp_ref = osr.SpatialReference()
    sp_ref.ImportFromWkt(proj_wkt)
    
    if (len(proj_wkt)>0)&(sp_ref.GetAttrValue('AUTHORITY',1)!=epsg):
        #check if the projection coincides with the epsg requested
        sp_ref_pyproj = sp_ref.ExportToProj4()
    
        newproj=pyproj.Proj(init='epsg:{}'.format(str(epsg))) 
        improj=pyproj.Proj(sp_ref_pyproj)
        l=[x_sw,y_sw,x_nw,y_nw,x_ne,y_ne,x_se,y_se]
        reprojected=[]
        [reprojected.extend(list(pyproj.transform(improj,newproj,*l[2*i:(2*i+2)]))) for i in range(len(l)//2)]
        x_sw,y_sw,x_nw,y_nw,x_ne,y_ne,x_se,y_se=reprojected


    return x_sw,y_sw,x_nw,y_nw,x_ne,y_ne,x_se,y_se,width,height


def trinagulate(data,corners=None,h=None,w=None,image_path=None,epsg=4326,flip_ax0=True):
    '''Converts pixel coordinates into geocoordinates.
    data is an array of arrays (coordinates);
    h,w are height and width of the image;
    image_path is path to the georeferenced image;
    epsg is the desired projection;
    flip_ax0 is the flag to flip x and y coordinates (try changing this if results are nans)
    '''
    
    
    if (not corners)|(h is None)|(w is None):
        print('Attempting to obtain metadata from image')
        corners_from_image = get_corners_from_image(image_path, epsg=epsg)
    
        if (h is None)|(w is None):
            w,h=corners_from_image[-2:]
        if not corners:
            corners=corners_from_image[:-2]
    
    x_sw,y_sw,x_nw,y_nw,x_ne,y_ne,x_se,y_se=corners
    points=[[h,0],[0,0],[0,w],[h,w]]
    values=[[x_sw,y_sw],[x_nw,y_nw],[x_ne,y_ne],[x_se,y_se]]
    g=scipy.interpolate.LinearNDInterpolator(points=points, values=values)
    
    
    long=[]
    for obj in data:
        long.append(obj.shape)
    
    long=np.sum(long,axis=0)
    long= np.argmax(long)==0
    
    
    out=[]
    for obj in data:
        if long:
            coords=np.vstack(obj)
        else:
            coords=np.hstack(obj).T
        if flip_ax0:
            coords=np.flip(coords,1)
        out.append(g(coords))
    
    return out




if __name__ == "__main__":    
    import pandas as pd
    import shapefiling 
    
    csv=pd.read_csv(os.path.join(os.getcwd(),'geocode_examples/NZ_Tauranga_1.csv'))
    
    impath=os.path.join(os.getcwd(),'geocode_examples/NZ_Tauranga_1.tif')
    
    objs=[]
    for c,obj in csv.groupby(by='Object',as_index=False):
            if obj['Label'].iloc[0]!='DT':
                continue
            coords=obj[['X','Y']].values
            objs.append(coords)
    
    
    output=trinagulate(objs,image_path=impath,flip_ax0=True)
    
    
    To_GeoFile_test=shapefiling.To_GeoFile(output,0)
    To_GeoFile_test.generate_geojson(os.path.join(os.getcwd(),'geocode_examples/output_test.geojson'))