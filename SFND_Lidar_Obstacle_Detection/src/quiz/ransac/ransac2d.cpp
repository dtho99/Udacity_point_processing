/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.2);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.2);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.2);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.2);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;	

	srand(time(NULL));
	
	// TODO: Fill in this function
	float x1, y1, x2, y2;
	float x, y;
	float a, b, c;
	int random_idx1, random_idx2;
	int size;
	float distance;
	int inlier_count;

	size = cloud->size();

	// For max iterations 
	for (int idx = 0; idx < maxIterations; idx++)
	{
		// Randomly sample subset and fit line


		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier

		// Return indicies of inliers from fitted line with most inliers		
		random_idx1 = rand() % size;
		x1 = cloud->points[random_idx1].x;
		y1 = cloud->points[random_idx1].y;

		random_idx2 = rand() % size;
		while(random_idx2 == random_idx1)
			random_idx2 = rand() % size;

		x2 = cloud->points[random_idx2].x;
		y2 = cloud->points[random_idx2].y;		

		a = y1 - y2;
		b = x2 - x1;
		c = (x1 * y2) - (x2 * y1);

				
		inliers.clear();
		for (int pts = 0; pts < cloud->size(); pts++)
		{
			
			{
				x = cloud->points[pts].x;
				y = cloud->points[pts].y;
				distance = fabs((a * x) + (b * y) + c) /
							sqrt((a*a) + sqrt(b*b));
				if (distance < distanceTol)
					inliers.insert(pts);
			}					
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			printf("%f %f %f\n", a, b, c);
		}

	}	
	
	return inliersResult;

}


std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	std::unordered_set<int> inliers;	

	srand(time(NULL));
	
	// TODO: Fill in this function
	float x1, y1, z1, x2, y2, z2, x3, y3, z3;
	float x, y, z;
	float a, b, c, d;
	float v1, v2;

	int random_idx1, random_idx2, random_idx3;
	int size;
	float distance;
	int inlier_count;

	size = cloud->size();

	// For max iterations 
	for (int idx = 0; idx < maxIterations; idx++)
	{
		// Randomly sample subset and fit line


		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier

		// Return indicies of inliers from fitted line with most inliers		
		random_idx1 = rand() % size;
		x1 = cloud->points[random_idx1].x;
		y1 = cloud->points[random_idx1].y;
		z1 = cloud->points[random_idx1].z;

		random_idx2 = rand() % size;
		while(random_idx2 == random_idx1)
			random_idx2 = rand() % size;

		x2 = cloud->points[random_idx2].x;
		y2 = cloud->points[random_idx2].y;
		z2 = cloud->points[random_idx2].z;		

		random_idx3 = rand() % size;
		while((random_idx3 == random_idx1) || (random_idx3 == random_idx1))
			random_idx3 = rand() % size;
		
		x3 = cloud->points[random_idx3].x;
		y3 = cloud->points[random_idx3].y;
		z3 = cloud->points[random_idx3].z;

		a = (y2-y1) * (z3-z1) - (z2-z1) * (y3-y1);
		b = (z2-z1) * (x3-x1) - (x2-x1) * (z3-z1);
		c = (x2-x1) * (y3-y1) - (y2-y1) * (x3-x1);
		d =  -(a*x1 + b*y1 + c*z1);

				
		inliers.clear();
		for (int pts = 0; pts < cloud->size(); pts++)
		{
			x = cloud->points[pts].x;
			y = cloud->points[pts].y;
			z = cloud->points[pts].z;

			distance = fabs((a*x) + (b*y) + (c*z) + d) /
						sqrt((a*a) + sqrt(b*b) + sqrt(c*c));

			if (distance < distanceTol)
				inliers.insert(pts);
				
		}

		if (inliers.size() > inliersResult.size())
		{
			inliersResult = inliers;
			printf("%f %f %f %f \n", a, b, c, d);
		}

	}	
	
	return inliersResult;

}


int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	//std::unordered_set<int> inliers = Ransac(cloud, 1000, 1);	


	// 3D plane
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = RansacPlane(cloud, 5000, .4);	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function


	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}


