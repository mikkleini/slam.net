using System;
using System.Collections.Generic;
using System.Numerics;
using System.Text;
using HectorSLAM.Map;

namespace HectorSLAM
{
    public class HectorSLAMProcessor
    {
        public IMapRepresentation MapRep;
        public Vector3 LastMapUpdatePose { get; protected set; }
        public Vector3 LastScanMatchPose { get; protected set; }
        public Matrix4x4 LastScanMatchCov { get; protected set; }

        public float paramMinDistanceDiffForMapUpdate;
        public float paramMinAngleDiffForMapUpdate;
        
        /*
        DrawInterface* drawInterface;
        HectorDebugInfoInterface* debugInterface;
        */

        public float ScaleToMap => MapRep.ScaleToMap;
        public int MapLevels => MapRep.MapLevels;


        public GridMap GetGridMap(int mapLevel = 0)
        {
            return MapRep.GetGridMap(mapLevel);
        }

        public void AddMapMutex(int i, IMapLocker mapMutex)
        {
            MapRep.AddMapMutex(i, mapMutex);
        }

        public IMapLocker GetMapMutex(int i)
        {
            return MapRep.GetMapMutex(i);
        }

        void setUpdateFactorFree(float free_factor) { MapRep.SetUpdateFactorFree(free_factor); }
        void setUpdateFactorOccupied(float occupied_factor) { MapRep.SetUpdateFactorOccupied(occupied_factor); }
        void setMapUpdateMinDistDiff(float minDist) { paramMinDistanceDiffForMapUpdate = minDist; }
        void setMapUpdateMinAngleDiff(float angleChange) { paramMinAngleDiffForMapUpdate = angleChange; }



        public HectorSLAMProcessor(float mapResolution, int mapSizeX, int mapSizeY, Vector2 startCoords, int multi_res_size, DrawInterface drawInterfaceIn = null, HectorDebugInfoInterface debugInterfaceIn = null)
        {
            mapRep = new MapRepMultiMap(mapResolution, mapSizeX, mapSizeY, multi_res_size, startCoords, drawInterfaceIn, debugInterfaceIn);

            Reset();

            this->setMapUpdateMinDistDiff(0.4f * 1.0f);
            this->setMapUpdateMinAngleDiff(0.13f * 1.0f);
        }


        public void Update(DataContainer dataContainer, Vector3 poseHintWorld, bool map_without_matching = false)
        {
            //std::cout << "\nph:\n" << poseHintWorld << "\n";

            Eigen::Vector3f newPoseEstimateWorld;

            if (!map_without_matching)
            {
                newPoseEstimateWorld = (MapRep.MatchData(poseHintWorld, dataContainer, LastScanMatchCov));
            }
            else
            {
                newPoseEstimateWorld = poseHintWorld;
            }

            LastScanMatchPose = newPoseEstimateWorld;

            //std::cout << "\nt1:\n" << newPoseEstimateWorld << "\n";

            //std::cout << "\n1";
            //std::cout << "\n" << lastScanMatchPose << "\n";
            if (util::poseDifferenceLargerThan(newPoseEstimateWorld, LastMapUpdatePose, paramMinDistanceDiffForMapUpdate, paramMinAngleDiffForMapUpdate) || map_without_matching)
            {
                MapRep.UpdateByScan(dataContainer, newPoseEstimateWorld);
                MapRep.OnMapUpdated();
                LastMapUpdatePose = newPoseEstimateWorld;
            }

            /*
            if (drawInterface)
            {
                const GridMap& gridMapRef(mapRep->getGridMap());
                drawInterface->setColor(1.0, 0.0, 0.0);
                drawInterface->setScale(0.15);

                drawInterface->drawPoint(gridMapRef.getWorldCoords(Vector2::Zero()));
                drawInterface->drawPoint(gridMapRef.getWorldCoords((gridMapRef.getMapDimensions().array()-1).cast<float>()));
                drawInterface->drawPoint(Vector2(1.0f, 1.0f));

                drawInterface->sendAndResetData();
            }

            if (debugInterface)
            {
                debugInterface->sendAndResetData();
            }*/
        }

        /// <summary>
        /// Reset
        /// </summary>
        public void Reset()
        {
            LastMapUpdatePose = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
            LastScanMatchPose = new Vector3(0.0f, 0.0f, 0.0f);

            //lastScanMatchPose.x() = -10.0f;
            //lastScanMatchPose.y() = -15.0f;
            //lastScanMatchPose.z() = M_PI*0.15f;

            MapRep.Reset();
        }
    }
}
