//
// Created by HuiWang on 2020/3/8.
//

#ifndef KNEE_MULTIMODALITY_REG_ITKIMAGETOVTKPOLYDATA_H
#define KNEE_MULTIMODALITY_REG_ITKIMAGETOVTKPOLYDATA_H

#include <itkImage.h>
#include <itkPoint.h>
#include <itkImageRegionConstIterator.h>
#include <itkImageFileReader.h>

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

template<typename ImageType>
vtkSmartPointer<vtkPolyData> itkImageTovtkPolyData(typename itk::ImageFileReader< ImageType >::Pointer &reader) {
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
            //std::cout << point << std::endl;

        }
        ++it;
    }
    polyData->SetPoints(points);
    return polyData;
}
#endif //KNEE_MULTIMODALITY_REG_ITKIMAGETOVTKPOLYDATA_H
