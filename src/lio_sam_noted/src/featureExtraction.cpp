/*
功能:
对经过畸变矫正的点云提取特征点
订阅:
1.订阅畸变矫正后的当前点云 (类型lio_sam::cloud_info)
发布:
1.发布当前点云提取到的特征集合 (类型lio_sam::cloud_info)
2.发布当前点云提取到的角点集合 (类型sensor_msgs::PointCloud2)
3.发布当前点云提取到的平面点集合 (类型sensor_msgs::PointCloud2)
*/

#include "utility.h"
#include "lio_sam/cloud_info.h"

        /*曲率数据类型 曲率+索引*/
        struct smoothness_t
        {
            float value;
            size_t ind;
        };

        struct by_value
        {
            bool operator()(smoothness_t const &left, smoothness_t const &right)
            {
                return left.value < right.value;
            }
        };

        class FeatureExtraction : public ParamServer
        {

        public:
            /*订阅: 雷达点云*/
            ros::Subscriber subLaserCloudInfo;
            /*发布: 雷达点云 角点 平面点*/
            ros::Publisher pubLaserCloudInfo;
            ros::Publisher pubCornerPoints;
            ros::Publisher pubSurfacePoints;
            /*点云:畸变矫正后点云、角点点云、平面点点云*/
            pcl::PointCloud<PointType>::Ptr extractedCloud;
            pcl::PointCloud<PointType>::Ptr cornerCloud;
            pcl::PointCloud<PointType>::Ptr surfaceCloud;
            /*体速滤波器*/
            pcl::VoxelGrid<PointType> downSizeFilter;
            /*单帧点云信息 note:lio_sam::cloud_info由msg定义*/
            lio_sam::cloud_info cloudInfo;
            std_msgs::Header cloudHeader;
            /*当前激光点云的曲率*/
            std::vector<smoothness_t> cloudSmoothness;
            float *cloudCurvature;    //曲率
            int *cloudNeighborPicked; // 1表示无法提取或已被提取，0表示尚未进行特征提取
            int *cloudLabel;          // 1表示角点

            /**
             * @brief  FeatureExtraction默认构造
             * @note 订阅+发布+变量参数初始化
             * @todo
             **/
            FeatureExtraction()
            {
                /*订阅畸变矫正后的点云*/
                /*Note:topic数据量过大时采用低延迟的TCP通讯 可参考:https://zhuanlan.zhihu.com/p/552346221*/
                subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
                /*发布提取的点云 特征点*/
                pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1);
                /*发布提取的点云 角点*/
                pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
                /*发布提取的点云 平面点*/
                pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
                /*初始化*/
                initializationValue();
            }

            /**
             * @brief 变量的初始化
             * @note 重置指针 指定容器大小 设置基本参数等
             * @todo
             **/
            void initializationValue()
            {
                /*指定 曲率容器的大小*/
                cloudSmoothness.resize(N_SCAN * Horizon_SCAN);
                /*设置体素滤波器 xyz方向滤波参数*/
                downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);
                /*重置点云指针 提取后的、角点的、平面的*/
                extractedCloud.reset(new pcl::PointCloud<PointType>());
                cornerCloud.reset(new pcl::PointCloud<PointType>());
                surfaceCloud.reset(new pcl::PointCloud<PointType>());
                /*数组记录 曲率/是否被筛选过/类型*/
                cloudCurvature = new float[N_SCAN * Horizon_SCAN];
                cloudNeighborPicked = new int[N_SCAN * Horizon_SCAN];
                cloudLabel = new int[N_SCAN * Horizon_SCAN];
            }

            /**
             * @brief 回调函数:执行该节点主要逻辑业务
             * @note
             * @todo
             **/
            void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn)
            {
                /*新消息拷贝到本地*/
                cloudInfo = *msgIn;
                cloudHeader = msgIn->header;
                /*将畸变矫正后点云信息转换格式*/
                pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction
                /*计算曲率*/
                calculateSmoothness();
                /*标记遮挡与平行点*/
                markOccludedPoints();
                /*提取特征点*/
                extractFeatures();
                /*发布特征点*/
                publishFeatureCloud();
            }

            /**
             * @brief 计算曲率
             * @note 见[LOAM] --> V.LIDAR ODOMETRY --> A.Feature  Point Extraction
             * @todo
             **/
            void calculateSmoothness()
            {
                /*
                排除前5点后5点计算曲率
                cloudSmoothness 带索引可以用它排序
                cloudLabel  取0 记未判断角点还是平面点
                cloudNeighborPicked 取0 记未进行特征提取
                cloudCurvature  仅数组记录
                */
                int cloudSize = extractedCloud->points.size();
                for (int i = 5; i < cloudSize - 5; i++)
                {
                    float diffRange = cloudInfo.pointRange[i - 5] + cloudInfo.pointRange[i - 4] + cloudInfo.pointRange[i - 3] + cloudInfo.pointRange[i - 2] + cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i] * 10 + cloudInfo.pointRange[i + 1] + cloudInfo.pointRange[i + 2] + cloudInfo.pointRange[i + 3] + cloudInfo.pointRange[i + 4] + cloudInfo.pointRange[i + 5];

                    cloudCurvature[i] = diffRange * diffRange; // diffX * diffX + diffY * diffY + diffZ * diffZ;

                    cloudNeighborPicked[i] = 0;
                    cloudLabel[i] = 0;
                    cloudSmoothness[i].value = cloudCurvature[i];
                    cloudSmoothness[i].ind = i;
                }
            }

            /**
             * @brief 标记是否属于被遮挡或是平行
             * @note
             * 见[LOAM] --> V.LIDAR ODOMETRY --> A.Feature  Point Extraction
             * 遮挡或者平行点稳定性差不提取
             * @todo
             **/
            void markOccludedPoints()
            {
                /*计算点云个数*/
                int cloudSize = extractedCloud->points.size();
                /*
                根据深度标记被遮挡点和平行点
                note:取得cloudSize - 6    2单元滑窗前后比较
                */
                for (int i = 5; i < cloudSize - 6; ++i)
                {
                    /*
                    partA:讨论遮挡情况
                    当前后索引在10以内时认为是属于同一条线
                    且在断点距离差超过0.3标记该点及相邻五点不再进行特征提取
                    */
                    float depth1 = cloudInfo.pointRange[i];
                    float depth2 = cloudInfo.pointRange[i + 1];
                    int columnDiff = std::abs(int(cloudInfo.pointColInd[i + 1] - cloudInfo.pointColInd[i]));

                    if (columnDiff < 10)
                    {
                        // 10 pixel diff in range image
                        if (depth1 - depth2 > 0.3)
                        {
                            cloudNeighborPicked[i - 5] = 1;
                            cloudNeighborPicked[i - 4] = 1;
                            cloudNeighborPicked[i - 3] = 1;
                            cloudNeighborPicked[i - 2] = 1;
                            cloudNeighborPicked[i - 1] = 1;
                            cloudNeighborPicked[i] = 1;
                        }
                        else if (depth2 - depth1 > 0.3)
                        {
                            cloudNeighborPicked[i + 1] = 1;
                            cloudNeighborPicked[i + 2] = 1;
                            cloudNeighborPicked[i + 3] = 1;
                            cloudNeighborPicked[i + 4] = 1;
                            cloudNeighborPicked[i + 5] = 1;
                            cloudNeighborPicked[i + 6] = 1;
                        }
                    }
                    /*
                    partB:讨论平行情况
                    根据深度差判断是否近似平行
                    */
                    float diff1 = std::abs(float(cloudInfo.pointRange[i - 1] - cloudInfo.pointRange[i]));
                    float diff2 = std::abs(float(cloudInfo.pointRange[i + 1] - cloudInfo.pointRange[i]));

                    if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                        cloudNeighborPicked[i] = 1;
                }
            }

            /**
             * @brief 特征点提取
             * @note
             * 见[LOAM] --> V.LIDAR ODOMETRY --> A.Feature  Point Extraction
             * @todo
             **/
            void extractFeatures()
            {
                /*清空、重置变量*/
                cornerCloud->clear();
                surfaceCloud->clear();

                pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
                pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

                /*遍历扫描线*/
                for (int i = 0; i < N_SCAN; i++)
                {
                    surfaceCloudScan->clear();

                    /*每线等分成6段*/
                    for (int j = 0; j < 6; j++)
                    {

                        int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                        int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                        if (sp >= ep)
                            continue;
                        /*依据曲率大小排序 降序排列*/
                        std::sort(cloudSmoothness.begin() + sp, cloudSmoothness.begin() + ep, by_value());

                        /*曲率从大至小遍历*/
                        int largestPickedNum = 0;
                        for (int k = ep; k >= sp; k--)
                        {
                            /*由曲率找索引*/
                            int ind = cloudSmoothness[k].ind;
                            /*激光点未被处理且曲率大于阈值认为是角点*/
                            if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                            {
                                largestPickedNum++;
                                if (largestPickedNum <= 20) //角点至多取20个
                                {
                                    cloudLabel[ind] = 1;
                                    cornerCloud->push_back(extractedCloud->points[ind]);
                                }
                                else
                                {
                                    break;
                                }
                                /*同一条扫描线上前后5点标记不再处理，避免特征聚集*/
                                cloudNeighborPicked[ind] = 1;
                                for (int l = 1; l <= 5; l++)
                                {
                                    int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                                    if (columnDiff > 10)
                                        break;
                                    cloudNeighborPicked[ind + l] = 1;
                                }
                                for (int l = -1; l >= -5; l--)
                                {
                                    int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                                    if (columnDiff > 10)
                                        break;
                                    cloudNeighborPicked[ind + l] = 1;
                                }
                            }
                        }

                        for (int k = sp; k <= ep; k++)
                        {
                            int ind = cloudSmoothness[k].ind;
                            if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                            {

                                cloudLabel[ind] = -1;
                                cloudNeighborPicked[ind] = 1;

                                for (int l = 1; l <= 5; l++)
                                {

                                    int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                                    if (columnDiff > 10)
                                        break;

                                    cloudNeighborPicked[ind + l] = 1;
                                }
                                for (int l = -1; l >= -5; l--)
                                {

                                    int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                                    if (columnDiff > 10)
                                        break;

                                    cloudNeighborPicked[ind + l] = 1;
                                }
                            }
                        }
                        /*未提取的点和平面点都认为是平面点，加入平面点集合*/
                        for (int k = sp; k <= ep; k++)
                        {
                            if (cloudLabel[k] <= 0)
                            {
                                surfaceCloudScan->push_back(extractedCloud->points[k]);
                            }
                        }
                    }
                    /*平面点比较多 降采样一下*/
                    surfaceCloudScanDS->clear();
                    downSizeFilter.setInputCloud(surfaceCloudScan);
                    downSizeFilter.filter(*surfaceCloudScanDS);
                    /*加入平面点集合*/
                    *surfaceCloud += *surfaceCloudScanDS;
                }
            }

            /**
             * @brief 释放指针
             * @note cloudInfo中的四个指针
             * @todo
             **/
            void freeCloudInfoMemory()
            {
                cloudInfo.startRingIndex.clear();
                cloudInfo.endRingIndex.clear();
                cloudInfo.pointColInd.clear();
                cloudInfo.pointRange.clear();
            }

            /**
             * @brief 特征点发布
             * @note
             * @todo
             **/
            void publishFeatureCloud()
            {
                /*释放内存*/
                freeCloudInfoMemory();
                /*发布提取到的特征点*/
                cloudInfo.cloud_corner = publishCloud(pubCornerPoints, cornerCloud, cloudHeader.stamp, lidarFrame);
                cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
                /*发布当前点点云信息 发布至mapOptimization*/
                pubLaserCloudInfo.publish(cloudInfo);
            }
        };

        int main(int argc, char **argv)
        {
            ros::init(argc, argv, "lio_sam");

            /*初始化*/
            FeatureExtraction FE;

            ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
            /*回调函数:执行算法逻辑 FeatureExtraction::laserCloudInfoHandler*/
            ros::spin();

            return 0;
        }
 