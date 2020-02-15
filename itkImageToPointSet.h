//
// Created by HuiWang on 2020/2/15.
//

#ifndef KNEE_MULTIMODALITY_REG_ITKIMAGETOPOINTSET_H
#define KNEE_MULTIMODALITY_REG_ITKIMAGETOPOINTSET_H

#include "itkImage.h"
#include "itkPointSet.h"
#include "itkImageRegionConstIterator.h"
#include "itkImageFileReader.h"

template<typename PointSetType, typename ImageType>
typename PointSetType::Pointer itkImageToPointSet(itk::ImageFileReader< ImageType > * reader) {
    typename PointSetType::Pointer pointSet = PointSetType::New();
    using IteratorType = itk::ImageRegionConstIterator< ImageType >;
    const ImageType * image = reader->GetOutput();
    IteratorType it( image, image->GetBufferedRegion() );
    it.GoToBegin();
    typename PointSetType::PointType point;
    unsigned long pointId = 0;
    unsigned long samples = 0;
    while( !it.IsAtEnd())
    {
        if(  it.Get() - 0 > 1e-2 )
        {
            if( samples % 100 == 0 )
            {
                // Convert the pixel position into a Point
                image->TransformIndexToPhysicalPoint( it.GetIndex() , point );
                pointSet->SetPoint( pointId, point );
//                std::cout << point << std::endl;
                // Transfer the pixel data to the value associated with the point.
                pointSet->SetPointData( pointId, it.Get() );
                ++pointId;
            }
            ++samples;
        }
        ++it;
    }
    std::cout << "Number Of Points = ";
    std::cout << pointSet->GetNumberOfPoints() << std::endl;
    return pointSet;
}


#endif //KNEE_MULTIMODALITY_REG_ITKIMAGETOPOINTSET_H
