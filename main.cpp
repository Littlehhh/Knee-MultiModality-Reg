#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"
#include "itkMeshFileReader.h"
#include "itkMeshFileWriter.h"

#include "itkImage.h"
#include "itkPointSet.h"
#include "itkImageRegionConstIterator.h"

#include "itkEuler3DTransform.h"
#include "itkEuclideanDistancePointMetric.h"
#include "itkLevenbergMarquardtOptimizer.h"
#include "itkPointSetToPointSetRegistrationMethod.h"

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


template<typename PointSetType, typename ImageType>
typename PointSetType::Pointer itkImageToPointSet(itk::ImageFileReader< ImageType > * reader) {
    typename PointSetType::Pointer  pointSet = PointSetType::New();
    using IteratorType = itk::ImageRegionConstIterator< ImageType >;
    const ImageType * image = reader->GetOutput();
    IteratorType it( image, image->GetBufferedRegion() );
    it.GoToBegin();
    using PointType = typename PointSetType::PointType;
    PointType point;
    unsigned long pointId = 0;
    while( !it.IsAtEnd())
    {
        if(  it.Get() - 0  > 1e-2 )
        {
            // Convert the pixel position into a Point
            image->TransformIndexToPhysicalPoint( it.GetIndex() , point );
            pointSet->SetPoint( pointId, point );
            // Transfer the pixel data to the value associated with the point.
            pointSet->SetPointData( pointId, it.Get() );
            ++pointId;
        }
        ++it;

    }
    std::cout << "Number Of Points = ";
    std::cout << pointSet->GetNumberOfPoints() << std::endl;
    return pointSet;
}


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
    using ImageReaderType = itk::ImageFileReader< ImageType >;

    const char * moving_image = argv[1];
    const char * fixed_image = argv[2];

    ImageReaderType::Pointer moving_reader = ImageReaderType::New();
    ImageReaderType::Pointer fixed_reader = ImageReaderType::New();
    moving_reader->SetFileName( moving_image  );
    fixed_reader->SetFileName( fixed_image  );
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
    std::cout << moving_image << std::endl;
    auto moving_PointSet = itkImageToPointSet<PointSetType, ImageType>(moving_reader);
    std::cout << fixed_image << std::endl;
    auto fixed_PointSet = itkImageToPointSet<PointSetType, ImageType>(fixed_reader);
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
    std::cout << "Stopping condition: " << optimizer->GetStopConditionDescription() << std::endl;
    return EXIT_SUCCESS;
}
