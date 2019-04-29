/* Usage:
 *
 * OrganizePointCloud<pcl::PointXYZ>(ros_msg, pcl_cloud, 32);
 *
 * Note: Explicitly declaring the type (in the above example: pcl::PointXYZ) is
 *       required for the templating to work properly.
 */

#ifndef ORGANIZE_VELODYNE_CLOUD_H
#define ORGANIZE_VELODYNE_CLOUD_H

// Eigen includes
#include <Eigen/StdVector>
#include <eigen3/Eigen/Geometry>          // Required for matrix math

// PCL includes
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// ROS includes
#include <sensor_msgs/PointCloud2.h>

// Misc includes
#include <velodyne_pointcloud/point_types.h>

using namespace pcl;
using namespace std;


/** @brief Takes in a partially organized cloud and fills the gaps in the
 *         current column with dummy points whose coordinates are NaN
 *  @param cloud     - A partially organized point cloud
 *  @param row_count - A vector which maintains the number of points in each
 *                     row. It's length is equal to the number of rows
 *  @param col       - The current column number, i.e. the number of points
 *                     that should be in each row. Each element in row_count
 *                     should be equal to this number
 */
template <typename PointType> void
FillWithNaNs(PointCloud<PointType>& cloud, vector<int>& row_count, int col)
{
  for (int k=0; k < row_count.size(); k++)
  {
    if (row_count[k] <= col)
    {
      PointType p_nan;
      p_nan.x = numeric_limits<float>::quiet_NaN();
      p_nan.y = numeric_limits<float>::quiet_NaN();
      p_nan.z = numeric_limits<float>::quiet_NaN();
      cloud(col, k) = p_nan;
      row_count[k]++;
    }
  }
}



/** @brief Takes in a ROS PointCloud2 object and outputs a PCL templated
 *         point cloud type, that has been organized. The input MUST be a
 *         cloud from a Velodyne LiDAR unit
 *  @param msg   - The input ROS point cloud message from a Velodyne LiDAR
 *  @param cloud - The organized cloud output
 *  @param rings - The number of rings in the Velodyne LiDAR unit
 */
template <typename PointType> void
OrganizePointCloud(sensor_msgs::PointCloud2 msg,
                    PointCloud<PointType>& cloud,
                    int rings)
{
  // Convert the PC to the PointXYZIR point type so we can access the ring
  PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_XYZIR
                            (new PointCloud<velodyne_pointcloud::PointXYZIR>);
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(msg, pcl_pc2);
  fromPCLPointCloud2(pcl_pc2, *cloud_XYZIR);

  /* Because not every row is complete, we can't know in advance how big the
     organized cloud needs to be, so we deliberately over-size by 30%. We then
     create a temporary cloud, SPECIFICALLY with the organized cloud
     constructor*/
  int columns = (cloud_XYZIR->points.size() / rings) * 2.0;
  PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_cloud_ptr (new PointCloud<pcl::PointXYZRGBNormal>);
  tmp_cloud_ptr->points.resize(columns * rings);
  tmp_cloud_ptr->height = static_cast<uint32_t>(rings);
  tmp_cloud_ptr->width = static_cast<uint32_t>(columns);
  tmp_cloud_ptr->is_dense = false;

  /* Iterate through the XYZIR points and fill the TMP cloud where the
     ring number determines the row the point is inserted into */
  int ring;
  int col = 0;
  vector<int> row_count(rings, 0);
  for (int i=0; i < cloud_XYZIR->points.size(); i++)
  {
    ring = cloud_XYZIR->points[i].ring;
    /* If true, we're just about to start processing the next 'column' of data
       and need to quickly fill all the holes in the current column with NaNs
       to act as dummy points - important for preserving the structure of the
       organized cloud */
    if (row_count[ring] > col)
    {
      FillWithNaNs(*tmp_cloud_ptr, row_count, col);
      col++;
    }
    PointType p;
    p.x = cloud_XYZIR->points[i].x;
    p.y = cloud_XYZIR->points[i].y;
    p.z = cloud_XYZIR->points[i].z;
    tmp_cloud_ptr->at(row_count[ring], ring) = p; // cloud(col, row)
    row_count[ring]++;
  }
  FillWithNaNs(*tmp_cloud_ptr, row_count, col); // Fill that last column

  /* Now we copy the organized tmp cloud to our output cloud, which we can now
     size correctly, knowing EXACTLY how many points are in each ring/row. But
     we have to do this point-by-point (rather than using memcpy) because we
     the points in memory that we want to keep are interpersed with points
     we want to leave behind */
  cloud = PointCloud<PointType>(row_count[0], rings);
  cloud.height = static_cast<uint32_t>(rings);
  cloud.width = static_cast<uint32_t>(row_count[0]);
  cloud.is_dense = false;
  for (int col=0; col < row_count[0]; col++)
  {
    for (int row=0; row < rings; row++)
    {
      cloud(col, row) = tmp_cloud_ptr->at(col, row);
    }
  }
}


#endif // ORGANIZE_VELODYNE_CLOUD_H
