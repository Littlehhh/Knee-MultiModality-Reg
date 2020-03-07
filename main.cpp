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


using PixelType = float;
constexpr unsigned int Dimension = 3;
using ImageType = itk::Image< PixelType, Dimension >;
using ImageReaderType = itk::ImageFileReader< ImageType >;
using FilterType = itk::ImageToVTKImageFilter<ImageType>;

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
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(fixed_reader->GetOutput());
//    filter->SetInput(moving_reader->GetOutput());
    try {
        moving_reader->Update();
        fixed_reader->Update();
    }
    catch (itk::ExceptionObject &err) {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    filter->Update();
    vtkImageData *fixed_image_vtk = filter->GetOutput();
    fixed_image_vtk->Print(std::cout);
    ImageType * itkimage = fixed_reader->GetOutput();
    auto itkmetadata = fixed_reader->GetMetaDataDictionary();
    // Convert the image to a polydata
    vtkSmartPointer<vtkImageDataGeometryFilter> imageDataGeometryFilter =
            vtkSmartPointer<vtkImageDataGeometryFilter>::New();
    imageDataGeometryFilter->SetInputData(fixed_image_vtk);
    imageDataGeometryFilter->Update();
    vtkSmartPointer<vtkPolyData> vtkPointSet = imageDataGeometryFilter->GetOutput();

    pcl::PolygonMesh pcl_data;
    pcl::VTKUtils::convertToPCL(vtkPointSet, pcl_data);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_data.cloud, cloud);
//    cloud.sensor_origin_ = nullptr;
//    cloud.sensor_orientation_ = nullptr;
//    cout << cloud.sensor_orientation_;
    pcl::UniformSampling<pcl::PointXYZ> pclfilter;
    pcl::PointCloud<pcl::PointXYZ>::Ptr test(&cloud);
    pclfilter.setInputCloud(test);
    pclfilter.setRadiusSearch(5);
    pclfilter.filter(*filteredCloud);

//    pcl::io::savePLYFile("MR.ply", *filteredCloud);
//    pcl::PCLPointCloud2Ptr test(&pcl_data.cloud);
//    pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.addPointCloud(filteredCloud, "sample cloud");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
//    viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer.addCoordinateSystem(0.5);
    viewer.initCameraParameters();
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
//        std::this_thread::sleep_for(100ms);
    }
    return 0;
}
////    viewer->addPointCloud<pcl::PointXYZ>(test);
//
//
//    // Create a mapper and actor
//    vtkSmartPointer<vtkPolyDataMapper> mapper =
//            vtkSmartPointer<vtkPolyDataMapper>::New();
//    mapper->SetInputConnection(imageDataGeometryFilter->GetOutputPort());
////    mapper->SetInputData(fixed_image_vtk);
//    vtkSmartPointer<vtkActor> actor =
//            vtkSmartPointer<vtkActor>::New();
//    actor->SetMapper(mapper);
//
//    // Visualization
//    vtkSmartPointer<vtkRenderer> renderer =
//            vtkSmartPointer<vtkRenderer>::New();
//    vtkSmartPointer<vtkRenderWindow> renderWindow =
//            vtkSmartPointer<vtkRenderWindow>::New();
//    renderWindow->AddRenderer(renderer);
//    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//            vtkSmartPointer<vtkRenderWindowInteractor>::New();
//    renderWindowInteractor->SetRenderWindow(renderWindow);
//    renderer->AddActor(actor);
//    renderer->SetBackground(0,0,0); // Background color white
//    renderWindow->Render();
//    renderWindow->SetSize(800, 600);
//    renderWindowInteractor->Start();
//
//}
//


