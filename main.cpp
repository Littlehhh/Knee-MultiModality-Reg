#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"

#include "itkImage.h"
#include "itkPointSet.h"
#include "itkImageRegionConstIterator.h"


int main( int argc, char * argv[] )
{
    // Verify the number of parameters in the command line
    if( argc < 2 )
    {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " inputImageFile  " << std::endl;
        return EXIT_FAILURE;
    }


    using PixelType = float;
    constexpr unsigned int Dimension = 3;

    using ImageType = itk::Image< PixelType, Dimension >;
    using PointSetType = itk::PointSet< PixelType, Dimension >;
    using ReaderType = itk::ImageFileReader< ImageType >;

    ReaderType::Pointer reader = ReaderType::New();

    const char * inputFilename  = argv[1];
    reader->SetFileName( inputFilename  );

    try
    {
        reader->Update();
    }
    catch( itk::ExceptionObject & err )
    {
        std::cout << "ExceptionObject caught !" << std::endl;
        std::cout << err << std::endl;
        return EXIT_FAILURE;
    }

    PointSetType::Pointer  pointSet = PointSetType::New();


    using IteratorType = itk::ImageRegionConstIterator< ImageType >;

    const ImageType * image = reader->GetOutput();

    IteratorType it( image, image->GetBufferedRegion() );

    it.GoToBegin();


    using PointType = PointSetType::PointType;
    PointType point;

    unsigned long pointId = 0;

    while( !it.IsAtEnd() )
    {

        // Convert the pixel position into a Point
        image->TransformIndexToPhysicalPoint( it.GetIndex() , point );
        pointSet->SetPoint( pointId, point );

        // Transfer the pixel data to the value associated with the point.
        pointSet->SetPointData( pointId, it.Get() );

        ++it;
        ++pointId;
    }


    std::cout << "Number Of Points = ";
    std::cout << pointSet->GetNumberOfPoints() << std::endl;


    // Software Guide : EndCodeSnippet
    return EXIT_SUCCESS;
}
