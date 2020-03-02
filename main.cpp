//
// Created by HuiWang on 2020/2/16.
//
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
//#include <pcl/visualization/pcl_visualizer.hpp>

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
    const char * moving_image_label = argv[1];
    const char * fixed_image_label = argv[2];
    ImageReaderType::Pointer moving_reader = ImageReaderType::New();
    ImageReaderType::Pointer fixed_reader = ImageReaderType::New();
    moving_reader->SetFileName( moving_image_label  );
    fixed_reader->SetFileName( fixed_image_label  );
    FilterType::Pointer filter = FilterType::New();
    filter->SetInput(fixed_reader->GetOutput());
    try
    {
        moving_reader->Update();
        fixed_reader->Update();
    }
    catch( itk::ExceptionObject & err )
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }
    filter->Update();
    vtkImageData * fixed_image_vtk = filter->GetOutput();
    fixed_image_vtk->Print(std::cout);

    // Convert the image to a polydata
    vtkSmartPointer<vtkImageDataGeometryFilter> imageDataGeometryFilter =
            vtkSmartPointer<vtkImageDataGeometryFilter>::New();
    imageDataGeometryFilter->SetInputData(fixed_image_vtk);
    imageDataGeometryFilter->Update();
    vtkSmartPointer<vtkPolyData> vtkPointSet = imageDataGeometryFilter->GetOutput();

    pcl::PolygonMesh pcl_data;
    pcl::VTKUtils::convertToPCL(vtkPointSet, pcl_data);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromPCLPointCloud2(pcl_data.cloud, cloud);
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr test( &cloud);
//    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//    viewer->addPointCloud<pcl::PointXYZ>(test);


    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper =
            vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(imageDataGeometryFilter->GetOutputPort());
//    mapper->SetInputData(fixed_image_vtk);
    vtkSmartPointer<vtkActor> actor =
            vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // Visualization
    vtkSmartPointer<vtkRenderer> renderer =
            vtkSmartPointer<vtkRenderer>::New();
    vtkSmartPointer<vtkRenderWindow> renderWindow =
            vtkSmartPointer<vtkRenderWindow>::New();
    renderWindow->AddRenderer(renderer);
    vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
            vtkSmartPointer<vtkRenderWindowInteractor>::New();
    renderWindowInteractor->SetRenderWindow(renderWindow);
    renderer->AddActor(actor);
    renderer->SetBackground(0,0,0); // Background color white
    renderWindow->Render();
    renderWindow->SetSize(800, 600);
    renderWindowInteractor->Start();

}

