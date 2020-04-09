//
// Created by HuiWang on 2020/2/16.
//

// use system eigen for vtk visualization
#define ITK_USE_SYSTEM_EIGEN

// cpp std
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
#include <pcl/features/pfh.h>

#include <pcl/registration/ndt.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>

// vtk header
#include <vtkSmartPointer.h>
#include <vtkNamedColors.h>

// project header
#include "itkImageTopclPointCloud.h"


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
pclPassThroughResult(PointCloudPtr &moving, PointCloudPtr &fixed){
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
pclICP(PointCloudPtr &moving, PointCloudPtr &fixed, Eigen::Matrix4f init_transform = Eigen::Matrix4f::Identity()){
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(moving);
    icp.setInputTarget(fixed);
    icp.setMaximumIterations(300);

    pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
    icp.align(*final, init_transform);
    std::cout << "Max Iterations:" << icp.getMaximumIterations() << endl;
    std::cout << "has converged:" << icp.hasConverged() << endl;
    std::cout << "score: " << icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;
    return final;
}
pcl::visualization::PCLVisualizer::Ptr
viewportsVis (PointCloudPtr &cloud1, PointCloudPtr &cloud2)
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
downSample(PointCloudPtr &input, float leafSize){
    pcl::VoxelGrid<PointType> voxelGrid;
    PointCloudPtr filtered(new PointCloudType);
    voxelGrid.setInputCloud(input);
    voxelGrid.setLeafSize(leafSize,leafSize,leafSize);
    voxelGrid.filter(*filtered);
    return  filtered;
}
pcl::visualization::PCLVisualizer::Ptr
NormalVis (PointCloudPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->initCameraParameters ();
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "cloud");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal> (cloud, normals,10, 5, "Normals");
    viewer->addCoordinateSystem (1.0);
    return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr
PCSVis (const PointCloudPtr& cloud, const pcl::PointCloud<pcl::Normal>::Ptr &normals,const pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr pcs)
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

void NDTReg(const PointCloudPtr &moving, const PointCloudPtr &fixed){
        // 初始化正态分布(NDT)对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // 根据输入数据的尺度设置NDT相关参数
    ndt.setTransformationEpsilon (0.01);// 为终止条件设置最小转换差异
    ndt.setStepSize (0.1);              // 为more-thuente线搜索设置最大步长
    ndt.setResolution (1.0);            // 设置NDT网格网格结构的分辨率（voxelgridcovariance）

    //以上参数在使用房间尺寸比例下运算比较好，但是如果需要处理例如一个咖啡杯子的扫描之类更小的物体，需要对参数进行很大程度的缩小

    //设置匹配迭代的最大次数，这个参数控制程序运行的最大迭代次数，一般来说这个限制值之前优化程序会在epsilon变换阀值下终止
    //添加最大迭代次数限制能够增加程序的鲁棒性阻止了它在错误的方向上运行时间过长
    ndt.setMaximumIterations (35);

    ndt.setInputSource (moving);  //源点云
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (fixed);  //目标点云

    // 设置使用机器人测距法得到的粗略初始变换矩阵结果
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

    // 计算需要的刚体变换以便将输入的源点云匹配到目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);

}

pcl::PointCloud<pcl::PFHSignature125>::Ptr
PFHcompute(const PointCloudPtr &cloud, float radius = 2){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(radius);
    ne.compute(*normals);

    // Setup the feature computation
    pcl::PFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::PFHSignature125> pfh_estimation;
    // Provide the original point cloud (without normals)
    pfh_estimation.setInputCloud (cloud);
    // Provide the point cloud with normals
    pfh_estimation.setInputNormals (normals);

    // pfh_estimation.setInputWithNormals (cloud, cloud_with_normals); PFHEstimation does not have this function
    // Use the same KdTree from the normal estimation
    pfh_estimation.setSearchMethod (tree);

    pcl::PointCloud<pcl::PFHSignature125>::Ptr pfh_features (new pcl::PointCloud<pcl::PFHSignature125>);

    pfh_estimation.setRadiusSearch (radius);

    // Actually compute the spin images
    pfh_estimation.compute (*pfh_features);
    return pfh_features;
}

pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
PCScompute(const PointCloudPtr &cloud){
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(2);
    ne.compute(*normals);

    // Setup the principal curvatures computation
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures>
            principal_curvatures_estimation;
    // Provide the original point cloud (without normals)
    principal_curvatures_estimation.setInputCloud (cloud);
    // Provide the point cloud with normals
    principal_curvatures_estimation.setInputNormals (normals);
    // Use the same KdTree from the normal estimation
    principal_curvatures_estimation.setSearchMethod (tree);
    principal_curvatures_estimation.setRadiusSearch (2);
    // Actually compute the principal curvatures
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr
    principal_curvatures (new pcl::PointCloud<pcl::PrincipalCurvatures> ());
    principal_curvatures_estimation.compute (*principal_curvatures);

    std::cout << "output points.size (): " << principal_curvatures->points.size() << std::endl;
    // Display and retrieve the shape context descriptor vector for the 0th point.
    pcl::PrincipalCurvatures descriptor = principal_curvatures->points[0];
    std::cout << descriptor << std::endl;
    return principal_curvatures;
}

std::vector<PointCloudPtr> clouds_vis;

int main( int argc, char * argv[] ) {
    // Verify the number of parameters in the command line
    if (argc < 3) {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " movingImageFile  fixedImageFile" << std::endl;
        return EXIT_FAILURE;
    }
//    const char *moving_image_label = argv[1];
//    const char *fixed_image_label = argv[2];
//    const char *ct_new = "/Users/hui/Projects/Knee-MultiModality-Reg/Data/CTContourLabelRemoveError.nii";
//    auto fixed_cloud = ReaditkImageToPCLPointCloud<ImageType>(moving_image_label);
//    auto moving_cloud = ReaditkImageToPCLPointCloud<ImageType>(fixed_image_label);
//    auto ct_new_cloud = ReaditkImageToPCLPointCloud<ImageType>(ct_new);
//    pclPassThroughResult(ct_new_cloud, fixed_cloud);

//    auto viewer = viewportsVis(moving_cloud, ct_new_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/MR.ply", *fixed_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/CT.ply", *moving_cloud);
//    pcl::io::savePLYFile<pcl::PointXYZ>("../Data/CT_new.ply", *ct_new_cloud);

    PointCloudPtr fixed(new PointCloudType);
    PointCloudPtr moving(new PointCloudType);
    pcl::io::loadPLYFile("../Data/MR_PT.ply", *fixed);
    pcl::io::loadPLYFile("../Data/CT_PT.ply", *moving);


    auto mf = downSample(moving, 1);
    auto ff = downSample(fixed, 1);

    auto mPFH = PFHcompute(mf,5);
    auto fPFH = PFHcompute(ff,5);

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::PFHSignature125> sac_ia;
    sac_ia.setInputSource(mf);
    sac_ia.setSourceFeatures(mPFH);
    sac_ia.setInputTarget(ff);
    sac_ia.setTargetFeatures(fPFH);
    PointCloudPtr align(new PointCloudType);
    //  sac_ia.setNumberOfSamples(20);  //设置每次迭代计算中使用的样本数量（可省）,可节省时间
    sac_ia.setCorrespondenceRandomness(6); //设置计算协方差时选择多少近邻点，该值越大，协防差越精确，但是计算效率越低.(可省)
    sac_ia.align(*align);

    pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125,pcl::PFHSignature125> crude_cor_est;
    pcl::CorrespondencesPtr cru_correspondences (new pcl::Correspondences);
    crude_cor_est.setInputSource(mPFH);
    crude_cor_est.setInputTarget(fPFH);
    //  crude_cor_est.determineCorrespondences(cru_correspondences);
//    crude_cor_est.determineReciprocalCorrespondences(*cru_correspondences);
//    cout<<"crude size is:"<<cru_correspondences->size()<<endl;


//    auto final_withInit = pclICP(mf, ff, sac_ia.getFinalTransformation());

//    auto final = pclICP(mf, ff);
//    clouds_vis.push_back(final);
//    clouds_vis.push_back(final_withInit);

//    clouds_vis.push_back(mf);
    clouds_vis.push_back(ff);
    clouds_vis.push_back(align);
//    auto viewer = PCSVis(ff, normals, principal_curvatures);
//    auto viewer = NormalVis(ff, normals);


    auto viewer = customColourVis(clouds_vis);
//    viewer->addCorrespondences<pcl::PointXYZ>(mf, ff, *cru_correspondences, "corr");
//    auto viewer = viewportsVis(ff, filtered);
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
//        std::this_thread::sleep_for(100ms);
    }
    return 0;
}


