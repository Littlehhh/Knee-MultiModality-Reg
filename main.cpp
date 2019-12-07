#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkGradientMagnitudeImageFilter.h"

int main( int argc, char **argv ) {
    typedef itk::Image<unsigned short,2>    	  ImageType;
    typedef itk::ImageFileReader<ImageType> 	  ReaderType;
    typedef itk::GradientMagnitudeImageFilter<
            ImageType,ImageType>     FilterType;

    ReaderType::Pointer reader = ReaderType::New();
    FilterType::Pointer filter = FilterType::New();

    reader->SetFileName( argv[1] );
    filter->SetInput( reader->GetOutput() );
    filter->Update();
    return 0;
}