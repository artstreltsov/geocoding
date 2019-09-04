"""
Created on Tue Aug 27 13:28:49 2019

@author: Artem Streltsov


This is a collection of classes for fast conversion of data into 
shapefiles/geojsons and back.

SEE BELOW FOR USAGE EXAMPLES
"""

import numpy as np
import json
import os, glob
from shapely.geometry import Point, LineString
from shapely.geometry.polygon import Polygon
from shapely.geometry.multipolygon import MultiPolygon
import pickle
import copy
import geojson
import shapely.wkt
from osgeo import gdal
if 'GDAL_DATA' not in os.environ:
    os.environ['GDAL_DATA'] = os.path.join(os.getcwd(),'gdal-data')

from osgeo import ogr
import osgeo.osr as osr


#CODE



wgs84_wkt = """
GEOGCS["WGS 84",
    DATUM["WGS_1984",
        SPHEROID["WGS 84",6378137,298.257223563,
            AUTHORITY["EPSG","7030"]],
        AUTHORITY["EPSG","6326"]],
    PRIMEM["Greenwich",0,
        AUTHORITY["EPSG","8901"]],
    UNIT["degree",0.01745329251994328,
        AUTHORITY["EPSG","9122"]],
    AUTHORITY["EPSG","4326"]]"""


class To_GeoFile(object):
    '''Creates a shapefile or a geojson.
    Data is a numpy array of objects and additional attributes;
    object_id is the index of the object coordinates in the numpy array, i.e.
    if the data array starts with the object geocoordinates the index is 0   
    '''
    def __init__(self, data,object_id):
        
        self.object_id=object_id
        #check if objects are shapely
        is_shapely=True
        try:
            _=data[0][object_id].geom_type
        except: #not shapely
            is_shapely=False
        
        self.data=copy.deepcopy(data)
        self.data = [list(entry) for entry in self.data]
        if not is_shapely: #infer shapely geometry
            for c,entry in enumerate(self.data):
                try: self.data[c][object_id]=self.infer_shapely(self.data[c][object_id])
                except: self.data[c]=self.infer_shapely(self.data[c])
    
    
    def infer_shapely(self,p):
        if isinstance(p, np.ndarray):
            p=p.tolist()
        inner_length=self.find_inner_length(p)
        for i in range(inner_length-1):
            p=p[0]
        if len(p)>1:
            p=Polygon(p)
        else:
            p=Point(p)
        return p
    
    def find_inner_length(self,l):
        if isinstance(l[0],list):
            return 1+self.find_inner_length(l[0])
        else:
            return 0
    
    
    def generate_shapefile(self,savename,layername,fields=None,fields_ids=None,object_type=ogr.wkbPolygon,epsg=4326):
        '''Call to convert data passed at class initialization into a shapefile.
        savename is the path to save under;
        layername is the name of the layer;
        fields are additional attribute NAMES if any;
        fields_ids are the indices the fields attributes are at in the data array;
        object_type is the type of the geoobject, pass an ogr object;
        epsg is the georeferencing to create shapefile as
        '''
        driver = ogr.GetDriverByName('Esri Shapefile')
        ds = driver.CreateDataSource(savename)
        srs = osr.SpatialReference()
        srs.ImportFromEPSG(epsg)
        
        
        
        layer = ds.CreateLayer(layername, srs, object_type)
        if fields is not None:
            t=self.data[0]
            field_types=[type(t_).__name__ for c,t_ in enumerate(t) if c in fields_ids]
            type_dict={'int':ogr.OFTInteger, 'int64':ogr.OFTInteger, 'float':ogr.OFTReal, 'str':ogr.OFTString, 'str_':ogr.OFTString, 'float64':ogr.OFTReal}
            [layer.CreateField(ogr.FieldDefn(f, type_dict[field_types[c]])) for c,f in enumerate(fields)]
        
        defn = layer.GetLayerDefn()
        
        
        for obj in self.data:
            feat = ogr.Feature(defn)
            if fields is not None:
                [feat.SetField(f, float(obj[fields_ids[c]])) if (np.isscalar(obj[fields_ids[c]]))&('str' not in str(type(obj[fields_ids[c]]))) else feat.SetField(f, obj[fields_ids[c]]) for c,f in enumerate(fields)]
            geom = ogr.CreateGeometryFromWkb(obj[self.object_id].wkb)
            feat.SetGeometry(geom)
            
            layer.CreateFeature(feat)
            feat = geom = None
        
        ds = layer = feat = geom = None
        
        return
    
    def generate_geojson(self,savename,fields=None,fields_ids=None):
        '''Call to convert data passed at class initialization into a geojson.
        savename is the path to save under;
        fields are additional attribute NAMES if any;
        fields_ids are the indices the fields attributes are at in the data array;
        '''
        features=[]
        
        if fields is not None:
            [features.append(geojson.Feature(properties={field: obj[fields_ids[c]] if not isinstance(obj[fields_ids[c]], (np.ndarray, np.generic)) else obj[fields_ids[c]].item() for c,field in enumerate(fields)},geometry=shapely.wkt.loads(obj[self.object_id].wkt))) for obj in self.data]
        else:
            try:[features.append(geojson.Feature(geometry=shapely.wkt.loads(obj[self.object_id].wkt))) for obj in self.data]
            except:[features.append(geojson.Feature(geometry=shapely.wkt.loads(obj.wkt))) for obj in self.data]
        
        f_coll = geojson.FeatureCollection(features)
        with open(savename, 'w') as f:
                 geojson.dump(f_coll, f)
        
        return






class From_GeoFile(object):
    '''This class converts shapefiles/geojsons into a Python readable list.
    '''
    
    def __init__(self, path):
        self.path=path
    
    def extract(self):
        if self.path.endswith('geojson'):
            return self.extract_geojson()
        elif self.path.endswith('shp'):
            return self.extract_shapefile()
    
    
    def extract_geojson(self):
        
        with open(self.path,'r') as f:
            features=json.load(f)
        
        out=[]
        for feat in features['features']:
            out.append([feat['geometry']['coordinates'],feat['properties']])
        
        return out
    
    
    
    def extract_shapefile(self):
        
        file=ogr.Open(self.path)
        layer = file.GetLayer(0)
        field_names = [field.name for field in layer.schema]
        
        out=[]
        for feat in layer:
            out.append([shapely.wkt.loads(feat.GetGeometryRef().ExportToWkt())]+[{f: feat.GetFieldAsString(f) for f in field_names}])
        
        return out   



if __name__ == "__main__":
    
    with open(os.path.join(os.getcwd(),'shapefiling_example/SDC_buildings.pkl'),'rb') as f:
        buildings=pickle.load(f)
    #buildings[0] is a list of field names
    #buildings[1:] is a list of coordinate arrays
        
    savename=os.path.join(os.getcwd(),'shapefiling_example/buildings_example.shp')
    epsg=4326
    layername='buildings'
    object_type=ogr.wkbPolygon
    object_id=0
    fields=buildings[0][2:]
    fields_ids=list(range(2,len(buildings[0])))
    data=buildings[1:]
    
    #create a shapefile
    To_GeoFile_test=To_GeoFile(data,0)
    To_GeoFile_test.generate_shapefile(savename,layername,fields=fields,fields_ids=fields_ids)
    
    #create a geojson
    savename_geojson=os.path.join(os.getcwd(),'shapefiling_example/buildings_example.geojson')
    To_GeoFile_test.generate_geojson(savename_geojson,fields,fields_ids)
    
    
    #load data into Python from shapefile
    From_GeoFile_test=From_GeoFile(savename).extract_shapefile()
    
    #load data into Python from geojson
    From_GeoFile_test2=From_GeoFile(savename_geojson).extract_geojson()
    