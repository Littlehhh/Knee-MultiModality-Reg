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


#include <pcl/filters/passthrough.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>


#include <pcl/io/ply_io.h>

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
using PointCloudConstPtr = PointCloudType::ConstPtr;


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
pcl::visualization::PCLVisualizer::Ptr
customColourVis (std::vector<PointCloudPtr> &clouds)
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
        std::string id = std::to_string(i);
        double r,g,b;
        nc->GetColor(names[i], r, g, b);
        PCLColor color(clouds[i], r*255, g*255, b*255);
        viewer->addPointCloud<pcl::PointXYZ> (clouds[i], color, id);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, id);
    }
    viewer->addCoordinateSystem (0.5);
    viewer->initCameraParameters ();
    return viewer;
}
void
pclPassThroughResult(PointCloudPtr moving, PointCloudPtr fixed){
    pcl::PassThrough<pcl::PointXYZ>::Ptr pass_z(new pcl::PassThrough<pcl::PointXYZ>);
    pass_z->setInputCloud(moving);
    pass_z->setFilterFieldName("z");
//    pass_z->setFilterLimits(-50,-15);
    pass_z->setFilterLimits(-65,-40);
    pass_z->setFilterLimitsNegative(false);
    PointCloudPtr moving_PT(new PointCloudType);
    pass_z->filter(*moving_PT);

    pass_z->setInputCloud(fixed);
    pass_z->setFilterFieldName("z");
    pass_z->setFilterLimits(-65,-20);
    pass_z->setFilterLimitsNegative(false);
    PointCloudPtr fixed_PT(new PointCloudType);
    pass_z->filter(*fixed_PT);

    pcl::io::savePLYFile("../Data/MR_PT.ply", *fixed_PT);
    pcl::io::savePLYFile("../Data/CT_PT.ply", *moving_PT);

}


pcl::PointCloud<pcl::PointXYZ>::Ptr
pclICP(PointCloudPtr moving, PointCloudPtr fixed){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(moving);
    icp.setInputTarget(fixed);
    icp.setMaximumIterations(200);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final);
    std::cout << "Max Iterations:" << icp.getMaximumIterations() << endl;
    std::cout << "has converged:" << icp.hasConverged() << endl;
    std::cout << "score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return final;
}
pcl::visualization::PCLVisualizer::Ptr
viewportsVis (PointCloudPtr cloud1, PointCloudPtr cloud2)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    int v1(0);  //创建新的视口
    viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);  //4个参数分别是X轴的最小值，最大值，Y轴的最小值，最大值，取值0-1，v1是标识
    viewer->setBackgroundColor (0, 0, 0, v1);    //设置视口的背景颜色
    viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);  //添加一个标签区别其他窗口  利用RGB颜色着色器并添加点云到视口中
    viewer->addPointCloud<pcl::PointXYZ> (cloud1, "sample cloud1", v1);   //对第二视口做同样的操作，使得做创建的点云分布于右半窗口，将该视口背景赋值于灰色，以便明显区别，虽然添加同样的点云，给点云自定义颜色着色
    int v2(0);
    viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
    viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
    viewer->addText("Radius: 0.01", 10, 10, "v2 text", v2);
    viewer->addPointCloud<pcl::PointXYZ> (cloud2, "sample cloud2", v2);  //为所有视口设置属性，
    viewer->addCoordinateSystem (1.0);  //添加法线  每个视图都有一组对应的法线
    return (viewer);
}
pcl::PointCloud<pcl::PointXYZ>::Ptr
downSample(PointCloudPtr input, float leafSize){
    pcl::VoxelGrid<PointType> voxelGrid;
    PointCloudPtr filtered(new PointCloudType);
    voxelGrid.setInputCloud(input);
    voxelGrid.setLeafSize(leafSize,leafSize,leafSize);
    voxelGrid.filter(*filtered);
    return  filtered;
}
pcl::visualization::PCLVisualizer::Ptr
NormalVis (PointCloudPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals,10, 5, "Normals");
    viewer->addCoordinateSystem (1.0);  //添加法线  每个视图都有一组对应的法线
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr
PCSVis (PointCloudPtr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals,pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>
    (cloud, normals,pcs,-10, 0.05, "pcs",0);
    viewer->addCoordinateSystem (1.0);
    return (viewer);
}


int main( int argc, char * argv[] ) {
    // Verify the number of parameters in the command line
    if (argc < 3) {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " movingImageFile  fixedImageFile" << std::endl;
        return EXIT_FAILURE;
    }
    const char *moving_image_label = argv[1];
    const char *fixed_image_label = argv[2];
    const char *ct_new = "/Users/hui/Projects/Knee-MultiModality-Reg/Data/CTContourLabelRemoveError.nii";
    std::vector<PointCloudPtr> c louds_vis;
    auto fixed_cloud = ReaditkImageToPCLPointCloud<ImageType>(moving_image_label);
    auto moving_cloud = ReaditkImageToPCLPointCloud<ImageType>(fixed_image_label);
    auto ct_new_cloud = ReaditkImageToPCLPointCloud<ImageType>(ct_new);
    pclPassThroughResult(ct_new_cloud, fixed_cloud);
//    clouds_vis.push_back(moving_cloud);
//    clouds_vis.push_back(ct_new_cloud);
//    auto viewer = viewportsVis(moving_cloud, ct_new_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/MR.ply", *fixed_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/CT.ply", *moving_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/CT_new.ply", *ct_new_cloud);

    PointCloudPtr fixed(new PointCloudType);
    PointCloudPtr moving(new PointCloudType);
    pcl::io::loadPLYFile("../Data/MR_PT.ply", *fixed);
    pcl::io::loadPLYFile("../Data/CT_PT.ply", *moving);
//    clouds_vis.push_back(fixed);
//    clouds_vis.push_back(moving);

    auto mf = downSample(moving, 2);
    auto ff = downSample(fixed, 2);
//    clouds_vis.push_back(mf);
    clouds_vis.push_back(ff);


//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
//    ne.setInputCloud(ff);
//    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
//    ne.setSearchMethod(tree);
//    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
//    ne.setRadiusSearch(2);
//    ne.compute(*normals);
//    // Setup the principal curvatures computation
//    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>
//            principal_curvatures_estimation;
//    // Provide the original point cloud (without normals)
//    principal_curvatures_estimation.setInputCloud (ff);
//    // Provide the point cloud with normals
//    principal_curvatures_estimation.setInputNormals (normals);
//    // Use the same KdTree from the normal estimation
//    principal_curvatures_estimation.setSearchMethod (tree);
//    principal_curvatures_estimation.setRadiusSearch (2);
//    // Actually compute the principal curvatures
//    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
//    principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
//    principal_curvatures_estimation.compute (*principal_curvatures);
//
//    std::cout << "output points.size (): " << principal_curvatures->points.size() << std::endl;
//    // Display and retrieve the shape context descriptor vector for the 0th point.
//    pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
//    std::cout << descriptor << std::endl;
//    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> statFilter;
//    PointCloudPtr filtered(new PointCloudType);
//    statFilter.setInputCloud(ff);
//    statFilter.setMeanK(10); //设置在进行统计时考虑查询点邻近点数
//    statFilter.setStddevMulThresh(0.1); //设置判断是否为离群点的阈值
//    statFilter.filter(*filtered);

    auto final = pclICP(mf, ff);
    clouds_vis.push_back(final);

//    auto viewer = PCSVis(ff, normals, principal_curvatures);
//    auto viewer = NormalVis(ff, normals);
    auto viewer = customColourVis(clouds_vis);
//    auto viewer = viewportsVis(ff, filtered);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
//        std::this_thread::sleep_for(100ms);
    }
    return 0;
}


