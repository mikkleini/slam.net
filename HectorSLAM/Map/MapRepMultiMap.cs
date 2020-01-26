using System;
using System.Collections.Generic;
using System.Text;

namespace HectorSLAM.Map
{
    public class MapRepMultiMap : IMapRepresentation
    {
        List<MapProcContainer> mapContainer;
        List<DataContainer> dataContainers;

        public float ScaleToMap { get; private set; }
        public int MapLevels { get; private set; }

        MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, int numDepth, Vector2 startCoords, DrawInterface* drawInterfaceIn, HectorDebugInfoInterface* debugInterfaceIn)
        {
            //unsigned int numDepth = 3;
            Eigen::Vector2i resolution(mapSizeX, mapSizeY);

                float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
                float mid_offset_x = totalMapSizeX * startCoords.x();

                float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
                float mid_offset_y = totalMapSizeY * startCoords.y();

            for (unsigned int i = 0; i<numDepth; ++i){
              std::cout << "HectorSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";
              GridMap* gridMap = new hectorslam::GridMap(mapResolution, resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y));
                OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
                ScanMatcher<OccGridMapUtilConfig<GridMap>>* scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap>>(drawInterfaceIn, debugInterfaceIn);

                mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

              resolution /= 2;
              mapResolution*=2.0f;
            }

            dataContainers.resize(numDepth-1);
        }

        public void Reset()
        {
            int size = mapContainer.size();

            for (unsigned int i = 0; i < size; ++i)
            {
                mapContainer[i].reset();
            }
        }

virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); };

  virtual int getMapLevels() const { return mapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); };

  virtual void addMapMutex(int i, MapLockerInterface* mapMutex)
{
    mapContainer[i].addMapMutex(mapMutex);
}

MapLockerInterface* getMapMutex(int i)
{
    return mapContainer[i].getMapMutex();
}

virtual void onMapUpdated()
{
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        mapContainer[i].resetCachedData();
    }
}

virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
{
    size_t size = mapContainer.size();

    Eigen::Vector3f tmp(beginEstimateWorld);

    for (int index = size - 1; index >= 0; --index)
    {
        //std::cout << " m " << i;
        if (index == 0)
        {
            tmp = (mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5));
        }
        else
        {
            dataContainers[index - 1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
            tmp = (mapContainer[index].matchData(tmp, dataContainers[index - 1], covMatrix, 3));
        }
    }
    return tmp;
}

virtual void updateByScan(const DataContainer& dataContainer, const Eigen::Vector3f& robotPoseWorld)
{
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        //std::cout << " u " <<  i;
        if (i == 0)
        {
            mapContainer[i].updateByScan(dataContainer, robotPoseWorld);
        }
        else
        {
            mapContainer[i].updateByScan(dataContainers[i - 1], robotPoseWorld);
        }
    }
    //std::cout << "\n";
}

virtual void setUpdateFactorFree(float free_factor)
{
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        GridMap & map = mapContainer[i].getGridMap();
        map.setUpdateFreeFactor(free_factor);
    }
}

virtual void setUpdateFactorOccupied(float occupied_factor)
{
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i)
    {
        GridMap & map = mapContainer[i].getGridMap();
        map.setUpdateOccupiedFactor(occupied_factor);
    }
}

    }
}
