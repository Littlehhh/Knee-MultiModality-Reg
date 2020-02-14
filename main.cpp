#include <fstream>

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"
#include "itkTransformFileWriter.h"
#include "itkCastImageFilter.h"
#include "itkResampleImageFilter.h"

#include "itkImage.h"
#include "itkPointSet.h"
#include "itkImageRegionConstIterator.h"

#include "itkEuler3DTransform.h"
#include "itkEuclideanDistancePointMetric.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkPointSetToPointSetRegistrationMethod.h"

#include "utils.h"


class CommandIterationUpdate : public itk::Command
{
public:
    using Self = CommandIterationUpdate;
    using Superclass = itk::Command;
    using Pointer = itk::SmartPointer<Self>;
    itkNewMacro( Self );

protected:
    CommandIterationUpdate() = default;

public:
    using OptimizerType = itk::LevenbergMarquardtOptimizer;
    using OptimizerPointer = const OptimizerType *;

    void Execute(itk::Object *caller, const itk::EventObject & event) override
    {
        Execute( (const itk::Object *)caller, event);
    }

    void Execute(const itk::Object * object, const itk::EventObject & event) override
    {
        auto optimizer = dynamic_cast< OptimizerPointer >( object );
        if( optimizer == nullptr )
        {
            itkExceptionMacro( "Could not cast optimizer." );
        }

        if( ! itk::IterationEvent().CheckEvent( &event ) )
        {
            return;
        }

        std::cout << "Value = " << optimizer->GetCachedValue() << std::endl;
        std::cout << "Position = "  << optimizer->GetCachedCurrentPosition();
        std::cout << std::endl << std::endl;
    }
};


int main( int argc, char * argv[] )
{
    // Verify the number of parameters in the command line
    if( argc < 3 )
    {
        std::cerr << "Usage: " << std::endl;
        std::cerr << argv[0] << " movingImageFile  fixedImageFile" << std::endl;
        return EXIT_FAILURE;
    }

    using PixelType = float;
    constexpr unsigned int Dimension = 3;
    using ImageType = itk::Image< PixelType, Dimension >;
    using PointSetType = itk::PointSet< PixelType, Dimension >;
    using PointsContainer = PointSetType::PointsContainer;
    using PointsIterator = PointsContainer::Iterator;
    using ImageReaderType = itk::ImageFileReader< ImageType >;


    const char * moving_image_label = argv[1];
    const char * fixed_image_label = argv[2];

    ImageReaderType::Pointer moving_reader = ImageReaderType::New();
    ImageReaderType::Pointer fixed_reader = ImageReaderType::New();
    moving_reader->SetFileName( moving_image_label  );
    fixed_reader->SetFileName( fixed_image_label  );
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
    std::cout << moving_image_label << std::endl;
    auto moving_PointSet = itkImageToPointSet<PointSetType, ImageType>(moving_reader);
    std::cout << fixed_image_label << std::endl;
    auto fixed_PointSet = itkImageToPointSet<PointSetType, ImageType>(fixed_reader);

    PointsContainer::Pointer Points = moving_PointSet->GetPoints();
    PointsIterator  pointsIterator = Points->Begin();
    std::ofstream outfile;
    outfile.open("moving.txt", std::ios::out | std::ios::trunc );

    while( pointsIterator != Points->End() )
    {
        itk::Point<float, 3> p = pointsIterator.Value();  // access the pixel data
        outfile << p << std::endl;              // save the pixel data
        ++pointsIterator;                      // advance to next pixel/point
    }
    outfile.close();

//
// registration
//
   using MetricType = itk::EuclideanDistancePointMetric<
           PointSetType, PointSetType >;

   MetricType::Pointer  metric = MetricType::New();
//
// Next, setup the tranform, optimizers, and registration.
//
   using TransformType = itk::Euler3DTransform< double >;
   TransformType::Pointer transform = TransformType::New();
   // Optimizer Type
   using OptimizerType = itk::LevenbergMarquardtOptimizer;
   OptimizerType::Pointer optimizer = OptimizerType::New();
   optimizer->SetUseCostFunctionGradient(false);
   // Registration Method
   using RegistrationType = itk::PointSetToPointSetRegistrationMethod<
           PointSetType, PointSetType >;
   RegistrationType::Pointer registration  = RegistrationType::New();
//
// Scale the translation components of the Transform in the Optimizer
//
   OptimizerType::ScalesType scales( transform->GetNumberOfParameters() );
//
// Next, set the scales and ranges for translations and rotations in the
// transform. Also, set the convergence criteria and number of iterations
// to be used by the optimizer.
//
   constexpr double translationScale = 1000.0; // dynamic range of translations
   constexpr double rotationScale = 1.0;       // dynamic range of rotations
   scales[0] = 1.0 / rotationScale;
   scales[1] = 1.0 / rotationScale;
   scales[2] = 1.0 / rotationScale;
   scales[3] = 1.0 / translationScale;
   scales[4] = 1.0 / translationScale;
   scales[5] = 1.0 / translationScale;

   unsigned long   numberOfIterations =  2000;
   double          gradientTolerance  =  1e-4;   // convergence criterion
   double          valueTolerance     =  1e-4;   // convergence criterion
   double          epsilonFunction    =  1e-5;   // convergence criterion

   optimizer->SetScales( scales );
   optimizer->SetNumberOfIterations( numberOfIterations );
   optimizer->SetValueTolerance( valueTolerance );
   optimizer->SetGradientTolerance( gradientTolerance );
   optimizer->SetEpsilonFunction( epsilonFunction );
//
// Here we start with an identity transform, although the user will usually
// be able to provide a better guess than this.
//
   transform->SetIdentity();
   registration->SetInitialTransformParameters( transform->GetParameters() );
//
// Connect all the components required for the registration.
//
   registration->SetMetric(        metric        );
   registration->SetOptimizer(     optimizer     );
   registration->SetTransform(     transform     );
   registration->SetFixedPointSet( fixed_PointSet );
   registration->SetMovingPointSet( moving_PointSet );
   // Connect an observer
   CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
   optimizer->AddObserver( itk::IterationEvent(), observer );
   try
   {
       registration->Update();
   }
   catch( itk::ExceptionObject & e )
   {
       std::cerr << e << std::endl;
       return EXIT_FAILURE;
   }
   std::cout << "Solution = " << transform->GetParameters() << std::endl;
   std::cout << "transform = " << transform << std::endl;
   std::cout << "Stopping condition: " << optimizer->GetStopConditionDescription() << std::endl;

   using TransformWriterType = itk::TransformFileWriterTemplate< double >;
   TransformWriterType::Pointer writer = TransformWriterType::New();

   //
   // We add a CompositeTransform with the
   // SetInput() function. This function takes any \doxygen{Transform}
   //

   writer->SetInput( transform );

   writer->SetFileName( "transformFileName.txt" );
   // Software Guide : EndCodeSnippet
   try
   {
       // Software Guide : BeginCodeSnippet
       writer->Update();
       // Software Guide : EndCodeSnippet
   }
   catch( itk::ExceptionObject & excp )
   {
       std::cerr << "Error while saving the transforms" << std::endl;
       std::cerr << excp << std::endl;
       return EXIT_FAILURE;
   }


// read origin image
//
//    const char * moving_image = "../Data/MR_up.nii";
//    const char * fixed_image = "../Data/CT_right_up.nii";
//    ImageReaderType::Pointer moving_image_reader = ImageReaderType::New();
//    ImageReaderType::Pointer fixed_image_reader = ImageReaderType::New();
//    moving_image_reader->SetFileName( moving_image  );
//    fixed_image_reader->SetFileName( fixed_image  );
//    try
//    {
//        moving_image_reader->Update();
//        fixed_image_reader->Update();
//    }
//    catch( itk::ExceptionObject & err )
//    {
//        std::cout << "ExceptionObject caught !" << std::endl;
//        std::cout << err << std::endl;
//        return EXIT_FAILURE;
//    }
//
//    using ResampleFilterType = itk::ResampleImageFilter<
//            ImageType,
//            ImageType >;
//
//    ResampleFilterType::Pointer resample = ResampleFilterType::New();
//
//    resample->SetTransform( registration->GetTransform() );
//    resample->SetInput( moving_image_reader->GetOutput() );
//
//    ImageType::Pointer fixedImage = fixed_image_reader->GetOutput();
//
//    resample->SetSize(    fixedImage->GetLargestPossibleRegion().GetSize() );
//    resample->SetOutputOrigin(  fixedImage->GetOrigin() );
//    resample->SetOutputSpacing( fixedImage->GetSpacing() );
//    resample->SetOutputDirection( fixedImage->GetDirection() );
//    resample->SetDefaultPixelValue( 100 );
//
//
//    // Prepare a writer and caster filters to send the resampled moving image to
//    // a file
//    //
//
//    using WriterType = itk::ImageFileWriter< ImageType >;
//    WriterType::Pointer  writer =  WriterType::New();
//    writer->SetFileName( argv[3] );
//    writer->SetInput( resample->GetOutput() );
//    writer->Update();


    return EXIT_SUCCESS;
}
