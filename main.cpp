//
// Created by HuiWang on 2020/2/16.
//
#define ITK_USE_SYSTEM_EIGEN
// uha

#include <iostream>
#include <string>
#include <vector>
// itk header
#include <itkImage.h>
#include <itkPointSet.h>
#include <itkImageFileReader.h>
#include <itkImageToVTKImageFilter.h>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include "itkImageTopclPointCloud.h"

#include <vtkSmartPointer.h>
#include <vtkNamedColors.h>

using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image< PixelType, Dimension >;
using PointType = pcl::PointXYZ;
using PointCloudType = pcl::PointCloud<PointType>;
using PointCloudPtr = PointCloudType::Ptr;


template<typename ImageType>
pcl::PointCloud<pcl::PointXYZ>::Ptr
ReaditkImageToPCLPointCloud(const char *file_name){
    using ImageReaderType = itk::ImageFileReader< ImageType >;
    typename ImageReaderType::Pointer reader = ImageReaderType::New();
    reader->SetFileName(file_name);
    try {
        reader->Update();
    }
    catch (itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
    }
    return itkImageTopclPointCloud<ImageType>(reader);
}

pcl::visualization::PCLVisualizer::Ptr customColourVis (std::vector<PointCloudPtr> &clouds)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    using PCLColor = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>;
    std::vector<std::string> names = {"red","green","blue","orange","coral"};
    viewer->setBackgroundColor (0, 0, 0);
    vtkSmartPointer<vtkNamedColors> nc = vtkSmartPointer<vtkNamedColors>::New();
    for(int i = 0; i < clouds.size(); ++i){
        double r,g,b;
        nc->GetColor(names[i], r, g, b);
        PCLColor color(clouds[i], r*255, g*255, b*255);
        viewer->addPointCloud<pcl::PointXYZ> (clouds[i], color, std::to_string(i));
    }
    viewer->addCoordinateSystem (0.5);
    viewer->initCameraParameters ();
    return viewer;
}

int main( int argc, char * argv[] ) {
    // Verify the number of parameters in the command line
    if (argc < 3) {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " movingImageFile  fixedImageFile" << std::endl;
        return EXIT_FAILURE;
    }
//    const char *moving_image_label = argv[1];
//    const char *fixed_image_label = argv[2];
//    auto fixed_cloud = ReaditkImageToPCLPointCloud<ImageType>(moving_image_label);
//    auto moving_cloud = ReaditkImageToPCLPointCloud<ImageType>(fixed_image_label);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/MR.ply", *fixed_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/CT.ply", *moving_cloud);
    std::vector<PointCloudPtr> clouds_vis;

    PointCloudPtr fixed(new PointCloudType);
    PointCloudPtr moving(new PointCloudType);
    pcl::io::loadPLYFile("../Data/MR.ply", *fixed);
    pcl::io::loadPLYFile("../Data/CT.ply", *moving);
//    clouds_vis.push_back(fixed);
//    clouds_vis.push_back(moving);

    pcl::PassThrough<pcl::PointXYZ>::Ptr pass_z(new pcl::PassThrough<pcl::PointXYZ>);
    pass_z->setInputCloud(moving);
    pass_z->setFilterFieldName("z");
    pass_z->setFilterLimits(-50,-15);
    pass_z->setFilterLimitsNegative(false);
    PointCloudPtr moving_PT(new PointCloudType);
    pass_z->filter(*moving_PT);
    clouds_vis.push_back(moving_PT);

    pass_z->setInputCloud(fixed);
    pass_z->setFilterFieldName("z");
    pass_z->setFilterLimits(-65,-30);
    pass_z->setFilterLimitsNegative(false);
    PointCloudPtr fixed_PT(new PointCloudType);
    pass_z->filter(*fixed_PT);
    clouds_vis.push_back(fixed_PT);


    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(moving_PT);
    icp.setInputTarget(fixed_PT);
    icp.setMaximumIterations(200);

    PointCloudPtr final(new PointCloudType);
    icp.align(*final);
    std::cout << "Max Iterations:" << icp.getMaximumIterations() << endl;
    std::cout << "has converged:" << icp.hasConverged() << endl;
    std::cout << "score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    clouds_vis.push_back(final);

    auto viewer = customColourVis(clouds_vis);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
//        std::this_thread::sleep_for(100ms);
    }
    return 0;
}


