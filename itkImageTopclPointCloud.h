//
// Created by HuiWang on 2020/3/8.
//

#ifndef KNEE_MULTIMODALITY_REG_ITKIMAGETOPCLPOINTCLOUD_H
#define KNEE_MULTIMODALITY_REG_ITKIMAGETOPCLPOINTCLOUD_H

#include <itkImage.h>
#include <itkPoint.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageFileReader.h>

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

#include <pcl/point_cloud.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

template<typename ImageType>
pcl::PointCloud<pcl::PointXYZ>::Ptr itkImageTopclPointCloud(typename itk::ImageFileReader< ImageType >::Pointer &reader) {
    vtkSmartPointer<vtkPoints> points =
            vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();
    using IteratorType = itk::ImageRegionConstIterator<ImageType>;
    const typename ImageType::Pointer image = reader->GetOutput();
    IteratorType it(image, image->GetBufferedRegion());
    it.GoToBegin();
    typename ImageType::PointType point;
    while (!it.IsAtEnd()) {
        if (it.Get() - 0 > 1e-2) {
            // Convert the pixel position into a Point
            image->TransformIndexToPhysicalPoint(it.GetIndex(), point);
            float x = point[0];
            float y = point[1];
            float z = point[2];
            points->InsertNextPoint(x, y, z);
        }
        ++it;
    }
    polyData->SetPoints(points);
    pcl::PolygonMesh::Ptr pcl_data(new pcl::PolygonMesh() );
    pcl::VTKUtils::convertToPCL(polyData, *pcl_data);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_data->cloud, *cloud);
    return cloud;
}


#endif //KNEE_MULTIMODALITY_REG_ITKIMAGETOPCLPOINTCLOUD_H
