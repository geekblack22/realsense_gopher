
/*
Project Name: ROS Node for Cluster Based Segmentation with PCL
Author: Sean Cassero 
Date: 2018
Availability:  git@github.com:jupidity/PCL-ROS-cluster-Segmentation.git

*/
#include <ros/ros.h>
#include <pcl/search/impl/search.hpp> 
#include <iostream>
// header for ROS core functionalities
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/range_image/range_image.h>
#include <pcl/common/transforms.h>
#include <pcl/segmentation/extract_clusters.h>
// including image message type to be able to receive and publish it
#include <sensor_msgs/PointCloud2.h>

#include <realsense_gopher/SegmentedClustersArray.h>

// // headers regarding the connection between opencv and ROS
// #include <image_transport/image_transport.hpp>
// #include "cv_bridge/cv_bridge.h"

// // OpenCV core functions and various image processing algorithms
// #include <opencv2/opencv.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/highgui/highgui.hpp>

using std::placeholders::_1;

// Defining a class that will be utilize in the "main" function
class CloudSegmenter 
{

	// Declaring pointer to the publisher and subscriber the publish and receive images.
	
	ros::Subscriber subscriptionPointCloud_;
	ros::Publisher PublisherPointClouds_;
  ros::Publisher PublisherPointCloudsList_;
	ros::Publisher PublisherFilteredPointCloud_;

  public:
	// Constructor of the class. The class is derived from the parent class "Node" of rclcpp and
	// the node is called "image_processor", and each time it receive a data, it will call the callbackImage() function
    	CloudSegmenter(ros::NodeHandle *nh)
	{
		
			try{

				subscriptionPointCloud_ = nh->subscribe("/camera/depth/color/points", 10,&CloudSegmenter::callbackPointCloud, this);
	}catch(char *excp){
		std::cout << "Oh no not my salad";
	}

	//defining the publisher: it will publish "Image" type data to the "output_image" topic
            
			PublisherPointClouds_ = nh->advertise<sensor_msgs::PointCloud2>("detected_objects",10);
      PublisherPointCloudsList_ = nh->advertise<realsense_gopher::SegmentedClustersArray>("detected_objects_list",10);

			PublisherFilteredPointCloud_ = nh->advertise<sensor_msgs::PointCloud2>("filtered_layer",10);

			
    }

  private:
	float getMaxHeight(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
          // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
        int pointCloudSize = cloud->size();
		
		float maxZValue = cloud->points[0].z;
        for(int nIndex = 0; nIndex < pointCloudSize; nIndex++){
			
		    cloud->points[nIndex].x;
            cloud->points[nIndex].y;
			float currentZValue = cloud->points[nIndex].z;
			if(currentZValue < maxZValue){
				maxZValue = currentZValue;
			}
        }
		return maxZValue;
    }

	void getTopLayer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
		
		int pointCloudSize = cloud->size();
		float maxHeight = getMaxHeight(cloud);
		
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		float heightDiff = .01;
		int count = 1;
		for(int nIndex = 0; nIndex < pointCloudSize; nIndex++){
			// RCLCPP_INFO_STREAM(get_logger(), maxHeight);
			if(cloud->points[nIndex].z < maxHeight + heightDiff){
				
				filteredCloud->points.push_back(cloud->points[nIndex]);
				count++;
				
			}
        }
		
		sensor_msgs::PointCloud2 cloud_msg;
      	
		pcl::toROSMsg(*filteredCloud, cloud_msg);
		

		
		cloud_msg.header.frame_id = "camera_link";
		PublisherFilteredPointCloud_.publish(cloud_msg);
		

		
	}
	


	void computeNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
		{
			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
			pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
			ne.setSearchMethod(tree);
			ne.setInputCloud(cloud);
			// Set the number of k nearest neighbors to use for the feature estimation.
			ne.setKSearch(25);
			ne.compute(*cloud_normals);
		}

  // Extract normals by index
  	void extractNormals(pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
		{
			pcl::ExtractIndices<pcl::Normal> extract_normals;
			extract_normals.setNegative(true);
			extract_normals.setInputCloud(cloud_normals);
			extract_normals.setIndices(inliers_plane);
			extract_normals.filter(*cloud_normals);
		}
	void removePlaneSurface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud_normals, pcl::PointIndices::Ptr inliers_plane)
		{
			// Find Plane
			pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
			segmentor.setOptimizeCoefficients(true);
			segmentor.setModelType(pcl::SACMODEL_NORMAL_PLANE);
			segmentor.setMethodType(pcl::SAC_RANSAC);

			Eigen::Vector3f axis = Eigen::Vector3f(0.0,0.0,1.0);
			segmentor.setAxis(axis);
			segmentor.setMaxIterations(1000);
			segmentor.setDistanceThreshold(0.155);
			segmentor.setEpsAngle(0.09);
			segmentor.setNormalDistanceWeight(0.1);
			segmentor.setInputCloud(cloud);
			segmentor.setInputNormals(cloud_normals);

			// Output plane
			pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients);
			segmentor.segment(*inliers_plane, *coefficients_plane);

			/* Extract the planar inliers from the input cloud */
			pcl::ExtractIndices<pcl::PointXYZRGB> extract_indices;
			extract_indices.setInputCloud(cloud);
			extract_indices.setIndices(inliers_plane);

			/* Remove the planar inliers, extract the rest */
			extract_indices.setNegative(true);
			extract_indices.filter(*cloud);
		}
   void extractCylinder(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::ModelCoefficients::Ptr coefficients_cylinder,
                       pcl::PointCloud<pcl::Normal>::Ptr cloud_normals)
  {
    // Create the segmentation object for cylinder segmentation and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> segmentor;
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    segmentor.setOptimizeCoefficients(true);
    segmentor.setModelType(pcl::SACMODEL_CYLINDER);
    segmentor.setMethodType(pcl::SAC_RANSAC);
    // Set the normal angular distance weight
    segmentor.setNormalDistanceWeight(0.1);
    // run at max 1000 iterations before giving up
    segmentor.setMaxIterations(100000);
    // tolerance for variation from model
    segmentor.setDistanceThreshold(0.15);
    // min max values of radius in meters to consider
    segmentor.setRadiusLimits(0.01, 5);
    segmentor.setInputCloud(cloud);
    segmentor.setInputNormals(cloud_normals);

    // Obtain the cylinder inliers and coefficients
    segmentor.segment(*inliers_cylinder, *coefficients_cylinder);

    // Extract the cylinder inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers_cylinder);
    extract.setNegative(false);
    extract.filter(*cloud);
  }
    void removeOutliers(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		// Create the segmentation object for cylinder segmentation and set all the parameters
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> outlier_filter;

		outlier_filter.setInputCloud(cloud);
		outlier_filter.setMeanK(10);
		outlier_filter.setStddevMulThresh(3);
		outlier_filter.filter(*cloud);
	}
    void passThroughFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{
		pcl::PassThrough<pcl::PointXYZRGB> pass;
		pass.setInputCloud(cloud);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(0.26, 1.5);
		pass.filter(*cloud);
	}
void callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr&  cloud_msg) {
		// Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2Ptr cloudFilteredPtr (cloud_filtered);


  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);


  // Perform voxel grid downsampling filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01, 0.01, 0.01);
  sor.filter (*cloudFilteredPtr);


  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr (xyz_cloud); // need a boost shared pointer for pcl function inputs

  // convert the pcl::PointCloud2 tpye to pcl::PointCloud<pcl::PointXYZRGB>
  pcl::fromPCLPointCloud2(*cloudFilteredPtr, *xyzCloudPtr);


  //perform passthrough filtering to remove table leg

  // create a pcl object to hold the passthrough filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered (xyz_cloud_filtered);

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud (xyzCloudPtr);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (.26, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*xyzCloudPtrFiltered);



  // create a pcl object to hold the ransac filtered results
  pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud_ransac_filtered = new pcl::PointCloud<pcl::PointXYZRGB>;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrRansacFiltered (xyz_cloud_ransac_filtered);


  // perform ransac planar filtration to remove table top
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZRGB> seg1;
  // Optional
  seg1.setOptimizeCoefficients (true);
  // Mandatory
  seg1.setModelType (pcl::SACMODEL_PLANE);
  seg1.setMethodType (pcl::SAC_RANSAC);
  seg1.setDistanceThreshold (0.04);

  seg1.setInputCloud (xyzCloudPtrFiltered);
  seg1.segment (*inliers, *coefficients);


  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  //extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setInputCloud (xyzCloudPtrFiltered);
  extract.setIndices (inliers);
  extract.setNegative (true);
  extract.filter (*xyzCloudPtrRansacFiltered);



  // perform euclidean cluster segmentation to seporate individual objects

  // Create the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (xyzCloudPtrRansacFiltered);

  // create the extraction object for the clusters
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  // specify euclidean cluster parameters
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (2500);
  ec.setSearchMethod (tree);
  ec.setInputCloud (xyzCloudPtrRansacFiltered);
  // exctract the indices pertaining to each cluster and store in a vector of pcl::PointIndices
  ec.extract (cluster_indices);

  // declare an instance of the SegmentedClustersArray message
  realsense_gopher::SegmentedClustersArray CloudClusters;

  // declare the output variable instances
  sensor_msgs::PointCloud2 output;
  pcl::PCLPointCloud2 outputPCL;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_point;
  pcl::PCLPointCloud2  final_pcl;
  sensor_msgs::PointCloud2 MergedCloud;
  
  // here, cluster_indices is a vector of indices for each cluster. iterate through each indices object to work with them seporately
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {

    // create a new clusterData message object
    //obj_recognition::ClusterData clusterData;


    // create a pcl object to hold the extracted cluster
    pcl::PointCloud<pcl::PointXYZRGB> *cluster = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusterPtr (cluster);

    // now we are in a vector of indices pertaining to a single cluster.
    // Assign each point corresponding to this cluster in xyzCloudPtrPassthroughFiltered a specific color for identification purposes
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
    {
      clusterPtr->points.push_back(xyzCloudPtrRansacFiltered->points[*pit]);

        }


   
    pcl_point += *clusterPtr;
    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *clusterPtr ,outputPCL);
    
   
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output);
	  output.header.frame_id = "camera_link";
    // add the cluster to the array message
    //clusterData.cluster = output;
    
    CloudClusters.clusters.push_back(output);
    //Merge metadata
   

  }
  // publish the clusters
  pcl::toPCLPointCloud2(pcl_point,final_pcl);
  pcl_conversions::fromPCL(final_pcl,MergedCloud);
  MergedCloud.header.frame_id = "camera_link";

  PublisherPointClouds_.publish(MergedCloud);
  PublisherPointCloudsList_.publish(CloudClusters);
}
 
	
	
	
    
};

int main(int argc, char * argv[])
{
	
		ros::init(argc, argv, "segment_cloud");
		ros::NodeHandle nh;
		CloudSegmenter nc = CloudSegmenter(&nh);
		ros::spin();
	

  return 0;
}