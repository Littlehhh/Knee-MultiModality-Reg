//
// Created by HuiWang on 2020/2/16.
//
#define ITK_USE_SYSTEM_EIGEN
// uha

#include <fstream>
#include <iostream>

// itk header
#include <itkImage.h>
#include <itkPointSet.h>
#include <itkImageFileReader.h>
#include <itkImageToVTKImageFilter.h>

// vtk header
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageDataGeometryFilter.h>

// PCL header
#include <pcl/point_cloud.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>

#include "itkImageTopclPointCloud.h"

using PixelType = float;
constexpr unsigned int Dimension = 3;


using ImageType = itk::Image< PixelType, Dimension >;
using ImageReaderType = itk::ImageFileReader< ImageType >;
using FilterType = itk::ImageToVTKImageFilter<ImageType>;
using PointSetType = itk::PointSet< PixelType, Dimension >;


int main( int argc, char * argv[] ) {
    // Verify the number of parameters in the command line
    if (argc < 3) {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " movingImageFile  fixedImageFile" << std::endl;
        return EXIT_FAILURE;
    }
    const char *moving_image_label = argv[1];
    const char *fixed_image_label = argv[2];
    ImageReaderType::Pointer moving_reader = ImageReaderType::New();
    ImageReaderType::Pointer fixed_reader = ImageReaderType::New();
    moving_reader->SetFileName(moving_image_label);
    fixed_reader->SetFileName(fixed_image_label);

    try {
        moving_reader->Update();
        fixed_reader->Update();
    }
    catch (itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }

    auto fixed_cloud = itkImageTopclPointCloud<ImageType>(fixed_reader);
    auto moving_cloud = itkImageTopclPointCloud<ImageType>(moving_reader);

    pcl::PassThrough<pcl::PointXYZ>::Ptr pass_z(new pcl::PassThrough<pcl::PointXYZ>);
    pass_z->setInputCloud(moving_cloud);
    pass_z->setFilterFieldName("z");
    //z轴区间设置
    pass_z->setFilterLimits(-50,-15);
    //设置为保留还是去除
    pass_z->setFilterLimitsNegative(false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fz(new pcl::PointCloud<pcl::PointXYZ>);
    pass_z->filter(*cloud_fz);

    moving_cloud = cloud_fz;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(moving_cloud);
    icp.setInputTarget(fixed_cloud);
    icp.setMaximumIterations(50);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final);
    std::cout << "Max Iterations:" << icp.getMaximumIterations() << endl;
    std::cout << "has converged:" << icp.hasConverged() << endl;
    std::cout << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
//    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::UniformSampling<pcl::PointXYZ> pclfilter;
//    pclfilter.setInputCloud(cloud);
//    pclfilter.setRadiusSearch(5);
//    pclfilter.filter(*filteredCloud);

//    pcl::io::savePLYFile<pcl::PointXYZ>("MR.ply", *cloud);

    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fixed_color(fixed_cloud, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> moving_color(moving_cloud, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(final, 255, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> fz_color(cloud_fz, 255, 0, 0);


    viewer.addPointCloud(fixed_cloud, fixed_color, "fixed cloud");
    viewer.addPointCloud(moving_cloud, moving_color, "moving cloud");
    viewer.addPointCloud(final, final_color, "final");
//    viewer.addPointCloud(cloud_fz, fz_color, "fz");

    viewer.setBackgroundColor(0, 0, 0);
    viewer.addCoordinateSystem(0.5);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
//        std::this_thread::sleep_for(100ms);
    }
    return 0;
}


