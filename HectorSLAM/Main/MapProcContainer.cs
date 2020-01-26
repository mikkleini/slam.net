using System;
using System.Collections.Generic;
using System.Text;
using HectorSLAM.Map;
using HectorSLAM.Matcher;

namespace HectorSLAM.Main
{
    public class MapProcContainer
    {
        public GridMap gridMap;
        //OccGridMapUtilConfig<GridMap>* gridMapUtil;
        ScanMatcher<OccGridMapUtilConfig<GridMap>> scanMatcher;
        IMapLocker mapMutex;

        public MapProcContainer(GridMap* gridMapIn, OccGridMapUtilConfig<GridMap>* gridMapUtilIn, ScanMatcher<OccGridMapUtilConfig<GridMap>>* scanMatcherIn)
    : gridMap(gridMapIn)
    , gridMapUtil(gridMapUtilIn)
    , scanMatcher(scanMatcherIn)
    , mapMutex(0)
            { }

            virtual ~MapProcContainer()
            { }

            void cleanup()
            {
                delete gridMap;
                delete gridMapUtil;
                delete scanMatcher;

                if (mapMutex)
                {
                    delete mapMutex;
                }
            }

            void reset()
            {
                gridMap->reset();
                gridMapUtil->resetCachedData();
            }

            void resetCachedData()
            {
                gridMapUtil->resetCachedData();
            }

            float getScaleToMap() const { return gridMap->getScaleToMap();
        };

        const GridMap& getGridMap() const { return *gridMap; };
    GridMap& getGridMap() { return *gridMap; };

    void addMapMutex(MapLockerInterface* mapMutexIn)
    {
        if (mapMutex)
        {
            delete mapMutex;
        }

        mapMutex = mapMutexIn;
    }

    MapLockerInterface* getMapMutex()
    {
        return mapMutex;
    }

    Vector3 matchData(const Vector3& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix, int maxIterations)
    {
        return scanMatcher->matchData(beginEstimateWorld, *gridMapUtil, dataContainer, covMatrix, maxIterations);
    }

    void updateByScan(const DataContainer& dataContainer, const Vector3& robotPoseWorld)
    {
        if (mapMutex)
        {
            mapMutex->lockMap();
        }

        gridMap->updateByScan(dataContainer, robotPoseWorld);

        if (mapMutex)
        {
            mapMutex->unlockMap();
        }
    }


    }
}
