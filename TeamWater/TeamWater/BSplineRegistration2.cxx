//	Registration method: Multi-stage B Spline Registration
//	
//	Transforms: STAGE 1: Versor3DRigidTransform 
//				STAGE 2: AffineTransform 
//				STAGE 3: BSplineTransform
//
//	Metric:    Mattes Mutual Information Image To Image Metric
//	Optimizer: Regular Step Gradient Descent
//	Interpolator: Linear Interpolate Image Function
//
//	Process: (from ITK software guide)
//
//	First the two images are roughly aligned by using a transform
//  initialization, then they are registered using a rigid transform.
//
//  That in turn, is used to initialize a registration with an affine transform.
//
//  The transform resulting from the affine registration is used as the bulk
//  transform of a BSplineTransform. 
//
//  The deformable registration is computed, and finally the resulting transform 
//  is used to resample the moving image.

#include "ITKExamples.h"

#include "itkImageRegistrationMethod.h"
#include "itkMattesMutualInformationImageToImageMetric.h"

#include "itkTimeProbesCollectorBase.h"
#include "itkMemoryProbesCollectorBase.h"

#include "itkCenteredTransformInitializer.h"
#include "itkVersorRigid3DTransform.h"
#include "itkAffineTransform.h"
#include "itkBSplineTransform.h"
#include "itkRegularStepGradientDescentOptimizer.h"

#include "itkBSplineResampleImageFunction.h"
#include "itkBSplineDecompositionImageFilter.h"

#include "itkImageFileReader.h"
#include "itkImageFileWriter.h"

#include "itkResampleImageFilter.h"
#include "itkCastImageFilter.h"
#include "itkSquaredDifferenceImageFilter.h"

//  The following section of code implements a Command observer
//  used to monitor the evolution of the registration process.
#include "itkCommand.h"
#include <itkCompositeTransform.h>
class CommandIterationUpdate : public itk::Command
{
public:
	typedef  CommandIterationUpdate   Self;
	typedef  itk::Command             Superclass;
	typedef itk::SmartPointer<Self>   Pointer;
	itkNewMacro(Self);

protected:
	CommandIterationUpdate() {};

public:
	// Pointer to regular step gradient descent optimizer.
	typedef itk::RegularStepGradientDescentOptimizer  OptimizerType;
	typedef   const OptimizerType *                   OptimizerPointer;

	void Execute(itk::Object *caller, const itk::EventObject & event) ITK_OVERRIDE
	{
		Execute((const itk::Object *)caller, event);
	}

	void Execute(const itk::Object * object, const itk::EventObject & event) ITK_OVERRIDE
	{
		// Print current optimizer iteration, value of the metric cost function & 
		// current position on the parameter space (commented out).
		OptimizerPointer optimizer = static_cast< OptimizerPointer >(object);
		std::cout << optimizer->GetCurrentIteration() << " = ";
		std::cout << optimizer->GetValue() << "  ";
		// std::cout << optimizer->GetCurrentPosition() << std::endl;
		std::cout << std::endl;
	}
};

void BSplineRegistration2(const char* fixedImageFile, const char* movingImageFile, const char* outputImageFile, const char* differenceImageFile) 
{
	// Declare image properties
	const    unsigned int    ImageDimension = 3;
	typedef  signed short    PixelType;

	typedef itk::Image< PixelType, ImageDimension >  FixedImageType;
	typedef itk::Image< PixelType, ImageDimension >  MovingImageType;

	// Template parameter for Affine and BSpline transform type declaration.
	const unsigned int SpaceDimension = ImageDimension;

	// Template parameter for BSpline transform declaration.
	const unsigned int SplineOrder = 3;
	typedef double CoordinateRepType;

	// Declare transformation types with template parameters.

	// First transformation.
	typedef itk::VersorRigid3DTransform< double > RigidTransformType;

	// Second transformation.
	typedef itk::AffineTransform< double, SpaceDimension > AffineTransformType;

	// Third transformation.
	typedef itk::BSplineTransform<CoordinateRepType, SpaceDimension, SplineOrder>  DeformableTransformType;

	// Declare CenteredTransformInitializer type. This is used to initialize the center of rotation
	// and the translation of transforms having the center of rotation among the parameters.
	typedef itk::CenteredTransformInitializer
		<RigidTransformType, FixedImageType, MovingImageType>  TransformInitializerType;

	typedef itk::RegularStepGradientDescentOptimizer			OptimizerType;

	typedef itk::MattesMutualInformationImageToImageMetric<FixedImageType,MovingImageType> MetricType;

	typedef itk::LinearInterpolateImageFunction<MovingImageType, double>    InterpolatorType;

	// The registration method type is instantiated using the types of the fixed and moving images.
	// This class is responsible for interconnecting all the components that we have declared so far.
	typedef itk::ImageRegistrationMethod<FixedImageType, MovingImageType>    RegistrationType;

	MetricType::Pointer         metric = MetricType::New();
	OptimizerType::Pointer      optimizer = OptimizerType::New();
	InterpolatorType::Pointer   interpolator = InterpolatorType::New();
	RegistrationType::Pointer   registration = RegistrationType::New();

	registration->SetMetric(metric);
	registration->SetOptimizer(optimizer);
	registration->SetInterpolator(interpolator);

	// Auxiliary identity transform. For debugging.
	typedef itk::IdentityTransform<double, SpaceDimension> IdentityTransformType;

	IdentityTransformType::Pointer identityTransform = IdentityTransformType::New();

	// Read the Fixed and Moving images.
	typedef itk::ImageFileReader< FixedImageType  > FixedImageReaderType;
	typedef itk::ImageFileReader< MovingImageType > MovingImageReaderType;

	FixedImageReaderType::Pointer  fixedImageReader = FixedImageReaderType::New();
	MovingImageReaderType::Pointer movingImageReader = MovingImageReaderType::New();

	fixedImageReader->SetFileName(fixedImageFile);		// originally argv[1]
	movingImageReader->SetFileName(movingImageFile);	// originally argv[2]

//  In this example, the input images are taken from readers. 
//  This requires the \doxygen{ImageRegistrationMethodv4} to acquire its inputs
//  from the output of the readers.	

//	The code below updates the readers in order to ensure that the image parameters
//  (size, origin and spacing) are valid when used to initialize the
//  transform.  We intend to use the center of the fixed image as the
//  rotation center and then use the vector between the fixed image center
//  and the moving image center as the initial translation to be applied
//  after the rotation.

	try
	{
		fixedImageReader->Update();
		movingImageReader->Update();
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	// fixed image is output of the fixed image reader.
	FixedImageType::ConstPointer fixedImage = fixedImageReader->GetOutput();

	registration->SetFixedImage(fixedImage);
	registration->SetMovingImage(movingImageReader->GetOutput());

	// Add a time and memory probes collector for profiling the computation time
	// of every stage.
	itk::TimeProbesCollectorBase chronometer;
	itk::MemoryProbesCollectorBase memorymeter;

	// Set up the metric parameters.

	// The Mattes Mutual Information metric requires the user to specify the 
	// number of bins used to compute the entropy.

	// In a typical application, 50 histogram bins are sufficient.
	// Note however, that the number of bins may have dramatic effects on the 
	// optimizer’s behavior.
	metric->SetNumberOfHistogramBins(50);

	// Possible to use largest region?
	FixedImageType::RegionType fixedRegion = fixedImage->GetBufferedRegion();

	const unsigned int numberOfPixels = fixedRegion.GetNumberOfPixels();

	metric->ReinitializeSeed(76926294);	// Mattes Mutual needs a seed.

				
	// Initialize a rigid transform by using Image Intensity Moments.

	// 	typedef itk::CenteredTransformInitializer
	//  <RigidTransformType, FixedImageType, MovingImageType>  TransformInitializerType;
	// Initializer for the first transform.
	TransformInitializerType::Pointer initializer = TransformInitializerType::New();

	// Versor3DRigidTransform. <double>
	RigidTransformType::Pointer  rigidTransform = RigidTransformType::New();

	// Connect rigid transform (VersorRigid3DTransform) to fixed & moving images.
	initializer->SetTransform(rigidTransform);
	initializer->SetFixedImage(fixedImageReader->GetOutput());
	initializer->SetMovingImage(movingImageReader->GetOutput());

	// Method to find geometric center of image. Other option is MomentsOn() for center of mass.
	initializer->GeometryOn();

	std::cout << "Starting Rigid Transform Initialization " << std::endl;

	memorymeter.Start("Rigid Initialization");
	chronometer.Start("Rigid Initialization");

	// Compute center of image. Resulting values passed directly to transform.
	initializer->InitializeTransform();

	chronometer.Stop("Rigid Initialization");
	memorymeter.Stop("Rigid Initialization");

	std::cout << "Rigid Transform Initialization completed" << std::endl;
	std::cout << std::endl;

	registration->SetFixedImageRegion(fixedRegion);
	registration->SetInitialTransformParameters(rigidTransform->GetParameters());
	registration->SetTransform(rigidTransform);

	//
	//  Define optimizer normaliztion to compensate for different dynamic range
	//  of rotations and translations.
	//
	typedef OptimizerType::ScalesType       OptimizerScalesType;

	// rigidTransform is Versor3DTransform, which has 6 parameters.
	OptimizerScalesType optimizerScales(rigidTransform->GetNumberOfParameters());
	const double translationScale = 1.0 / 1000.0;

	// Set the 6 parameters.
	// First 3 params define versor, last 3 define the translation in each dimension.
	optimizerScales[0] = 1.0;
	optimizerScales[1] = 1.0;
	optimizerScales[2] = 5.0;
	optimizerScales[3] = translationScale;
	optimizerScales[4] = translationScale;
	optimizerScales[5] = translationScale;

	optimizer->SetScales(optimizerScales);

	optimizer->SetMaximumStepLength(0.1200); // originally 0.2000
	optimizer->SetMinimumStepLength(0.00025);  // originally 0.0001

	optimizer->SetNumberOfIterations(500);

	//
	// The rigid transform has 6 parameters we use therefore a few samples to run
	// this stage.
	//
	// Regulating the number of samples in the Metric is equivalent to performing
	// multi-resolution registration because it is indeed a sub-sampling of the
	// image.

	metric->SetNumberOfSpatialSamples(10000L); // originally 10000L

	//
	// Create the Command observer and register it with the optimizer.
	//
	CommandIterationUpdate::Pointer observer = CommandIterationUpdate::New();
	optimizer->AddObserver(itk::IterationEvent(), observer);


	std::cout << "Starting Rigid Registration " << std::endl;

	try
	{
		memorymeter.Start("Rigid Registration");
		chronometer.Start("Rigid Registration");

		registration->Update();

		chronometer.Stop("Rigid Registration");
		memorymeter.Stop("Rigid Registration");

		std::cout << "Optimizer stop condition = "
			<< registration->GetOptimizer()->GetStopConditionDescription()
			<< std::endl;
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	std::cout << "Rigid Registration completed" << std::endl;
	std::cout << std::endl;

	// Get output transformation from registration.
	rigidTransform->SetParameters(registration->GetLastTransformParameters());

//------------------------------------------------------------------------------------------------

	//  Perform Affine Registration
	AffineTransformType::Pointer  affineTransform = AffineTransformType::New();

	affineTransform->SetCenter(rigidTransform->GetCenter());
	affineTransform->SetTranslation(rigidTransform->GetTranslation());
	affineTransform->SetMatrix(rigidTransform->GetMatrix());

	registration->SetTransform(affineTransform);
	registration->SetInitialTransformParameters(affineTransform->GetParameters());

	optimizerScales = OptimizerScalesType(affineTransform->GetNumberOfParameters());

	// Affine parameters. 12 parameters. (N + 1) * N = (3+1) * 3 = 12.
	// The first N × N parameters define the matrix in column - major order
	// (where the column index varies the fastest)
	// All originally 1.0.
	optimizerScales[0] = 0.1;
	optimizerScales[1] = 0.5;
	optimizerScales[2] = 1.0;
	optimizerScales[3] = 0.5;
	optimizerScales[4] = 1.0;
	optimizerScales[5] = 1.0;
	optimizerScales[6] = 1.0;
	optimizerScales[7] = 1.0;
	optimizerScales[8] = 0.1;

	// The last N parameters define the translations for each dimension.
	optimizerScales[9] = translationScale;
	optimizerScales[10] = translationScale;
	optimizerScales[11] = translationScale;

	optimizer->SetScales(optimizerScales);

	optimizer->SetMaximumStepLength(0.1000);	// Originally 0.2000
	optimizer->SetMinimumStepLength(0.0002);	// Originally 0.0001

	optimizer->SetNumberOfIterations(200);

	//
	// The Affine transform has 12 parameters we use therefore a more samples to run
	// this stage.
	//
	// Regulating the number of samples in the Metric is equivalent to performing
	// multi-resolution registration because it is indeed a sub-sampling of the
	// image.
	metric->SetNumberOfSpatialSamples(50000L);	// originally 50000L

	std::cout << "Starting Affine Registration " << std::endl;

	try
	{
		memorymeter.Start("Affine Registration");
		chronometer.Start("Affine Registration");

		registration->Update();

		chronometer.Stop("Affine Registration");
		memorymeter.Stop("Affine Registration");
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	std::cout << "Affine Registration completed" << std::endl;
	std::cout << std::endl;

	affineTransform->SetParameters(registration->GetLastTransformParameters());

//-----------------------------------------------------------------------------------------------------

	
	//  Perform Deformable Registration

	// typedef itk::BSplineTransform<CoordinateRepType, SpaceDimension, SplineOrder>  DeformableTransformType;
	DeformableTransformType::Pointer  bsplineTransformCoarse = DeformableTransformType::New();

	unsigned int numberOfGridNodesInOneDimensionCoarse = 5;

	DeformableTransformType::PhysicalDimensionsType   fixedPhysicalDimensions;
	DeformableTransformType::MeshSizeType             meshSize;
	DeformableTransformType::OriginType               fixedOrigin;

	for (unsigned int i = 0; i< SpaceDimension; i++)
	{
		fixedOrigin[i] = fixedImage->GetOrigin()[i];

		fixedPhysicalDimensions[i] = fixedImage->GetSpacing()[i] *
			static_cast<double>(fixedImage->GetLargestPossibleRegion().GetSize()[i] - 1);
	}
	meshSize.Fill(numberOfGridNodesInOneDimensionCoarse - SplineOrder);

	bsplineTransformCoarse->SetTransformDomainOrigin(fixedOrigin);
	bsplineTransformCoarse->SetTransformDomainPhysicalDimensions(fixedPhysicalDimensions);
	bsplineTransformCoarse->SetTransformDomainMeshSize(meshSize);
	bsplineTransformCoarse->SetTransformDomainDirection(fixedImage->GetDirection());

	typedef DeformableTransformType::ParametersType     ParametersType;

	// Scale the translation components of the Transform in the Optimizer
	unsigned int numberOfBSplineParameters = bsplineTransformCoarse->GetNumberOfParameters();

	// 375 parameters
	std::cout << "BSpline params = " << numberOfBSplineParameters << std::endl;

	optimizerScales = OptimizerScalesType(numberOfBSplineParameters);
	
	optimizerScales.Fill(0.5);

	optimizer->SetScales(optimizerScales);

	ParametersType initialDeformableTransformParameters(numberOfBSplineParameters);
	initialDeformableTransformParameters.Fill(0.0);

	bsplineTransformCoarse->SetParameters(initialDeformableTransformParameters);

	registration->SetInitialTransformParameters(bsplineTransformCoarse->GetParameters());
	registration->SetTransform(bsplineTransformCoarse);

	// Software Guide : EndCodeSnippet


	//  Software Guide : BeginLatex
	//
	//  Next we set the parameters of the RegularStepGradientDescentOptimizer object.
	//
	//  Software Guide : EndLatex


	// Software Guide : BeginCodeSnippet
	optimizer->SetMaximumStepLength(20.0);	// Originally 10.0
	optimizer->SetMinimumStepLength(0.1);	// Originally 0.01

	optimizer->SetRelaxationFactor(0.9);
	optimizer->SetNumberOfIterations(200);
	// Software Guide : EndCodeSnippet

	//
	// The BSpline transform has a large number of parameters, we use therefore a
	// much larger number of samples to run this stage.
	//
	// Regulating the number of samples in the Metric is equivalent to performing
	// multi-resolution registration because it is indeed a sub-sampling of the
	// image.
	metric->SetNumberOfSpatialSamples(numberOfBSplineParameters * 100);

	std::cout << std::endl << "Starting Deformable Registration Coarse Grid" << std::endl;

	try
	{
		memorymeter.Start("Deformable Registration Coarse");
		chronometer.Start("Deformable Registration Coarse");

		registration->Update();

		chronometer.Stop("Deformable Registration Coarse");
		memorymeter.Stop("Deformable Registration Coarse");
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	std::cout << "Deformable Registration Coarse Grid completed" << std::endl;
	std::cout << std::endl;

	OptimizerType::ParametersType finalParameters =
		registration->GetLastTransformParameters();

	bsplineTransformCoarse->SetParameters(finalParameters);
	
	//  Software Guide : BeginLatex
	//
	//  Once the registration has finished with the low resolution grid, we
	//  proceed to instantiate a higher resolution
	//  \code{BSplineTransform}.
	//
	//  Software Guide : EndLatex

	DeformableTransformType::Pointer  bsplineTransformFine = DeformableTransformType::New();

	unsigned int numberOfGridNodesInOneDimensionFine = 5;

	meshSize.Fill( numberOfGridNodesInOneDimensionFine - SplineOrder );

	bsplineTransformFine->SetTransformDomainOrigin( fixedOrigin );
	bsplineTransformFine->SetTransformDomainPhysicalDimensions(
	fixedPhysicalDimensions );
	bsplineTransformFine->SetTransformDomainMeshSize( meshSize );
	bsplineTransformFine->SetTransformDomainDirection(
	fixedImage->GetDirection() );

	numberOfBSplineParameters = bsplineTransformFine->GetNumberOfParameters();

	ParametersType parametersHigh( numberOfBSplineParameters );
	parametersHigh.Fill( 0.0 );

	//  Software Guide : BeginLatex
	//
	//  Now we need to initialize the BSpline coefficients of the higher resolution
	//  transform. This is done by first computing the actual deformation field
	//  at the higher resolution from the lower resolution BSpline coefficients.
	//  Then a BSpline decomposition is done to obtain the BSpline coefficient of
	//  the higher resolution transform.
	//
	//  Software Guide : EndLatex

	unsigned int counter = 0;

	for ( unsigned int k = 0; k < SpaceDimension; k++ )
	{
	typedef DeformableTransformType::ImageType ParametersImageType;
	typedef itk::ResampleImageFilter<ParametersImageType,ParametersImageType> ResamplerType;
	ResamplerType::Pointer upsampler = ResamplerType::New();

	typedef itk::BSplineResampleImageFunction<ParametersImageType,double> FunctionType;
	FunctionType::Pointer function = FunctionType::New();

	upsampler->SetInput( bsplineTransformCoarse->GetCoefficientImages()[k] );
	upsampler->SetInterpolator( function );
	upsampler->SetTransform( identityTransform );
	upsampler->SetSize( bsplineTransformFine->GetCoefficientImages()[k]->
	GetLargestPossibleRegion().GetSize() );
	upsampler->SetOutputSpacing( bsplineTransformFine->GetCoefficientImages()[k]->
	GetSpacing() );
	upsampler->SetOutputOrigin( bsplineTransformFine->GetCoefficientImages()[k]->
	GetOrigin() );

	typedef itk::BSplineDecompositionImageFilter<ParametersImageType,ParametersImageType>
	DecompositionType;
	DecompositionType::Pointer decomposition = DecompositionType::New();

	decomposition->SetSplineOrder( SplineOrder );
	decomposition->SetInput( upsampler->GetOutput() );
	decomposition->Update();

	ParametersImageType::Pointer newCoefficients = decomposition->GetOutput();

	// copy the coefficients into the parameter array
	typedef itk::ImageRegionIterator<ParametersImageType> Iterator;
	Iterator it( newCoefficients, bsplineTransformFine->GetCoefficientImages()[k]->
	GetLargestPossibleRegion() );
	while ( !it.IsAtEnd() )
	{
	parametersHigh[ counter++ ] = it.Get();
	++it;
	}

	}


	optimizerScales = OptimizerScalesType( numberOfBSplineParameters );
	optimizerScales.Fill( 1.0 );

	optimizer->SetScales( optimizerScales );

	bsplineTransformFine->SetParameters( parametersHigh );

	//  Software Guide : BeginLatex
	//
	//  We now pass the parameters of the high resolution transform as the initial
	//  parameters to be used in a second stage of the registration process.
	//
	//  Software Guide : EndLatex

	std::cout << "Starting Registration with high resolution transform" << std::endl;

	// Software Guide : BeginCodeSnippet
	registration->SetInitialTransformParameters(
	bsplineTransformFine->GetParameters() );
	registration->SetTransform( bsplineTransformFine );
	//
	// The BSpline transform at fine scale has a very large number of parameters,
	// we use therefore a much larger number of samples to run this stage. In
	// this case, however, the number of transform parameters is closer to the
	// number of pixels in the image. Therefore we use the geometric mean of the
	// two numbers to ensure that the number of samples is larger than the number
	// of transform parameters and smaller than the number of samples.
	//
	// Regulating the number of samples in the Metric is equivalent to performing
	// multi-resolution registration because it is indeed a sub-sampling of the
	// image.
	const unsigned long numberOfSamples =
	static_cast<unsigned long>(
	std::sqrt( static_cast<double>( numberOfBSplineParameters ) *
	static_cast<double>( numberOfPixels ) ) );
	metric->SetNumberOfSpatialSamples( numberOfSamples );

	try
	{
	memorymeter.Start( "Deformable Registration Fine" );
	chronometer.Start( "Deformable Registration Fine" );
	registration->Update();
	chronometer.Stop( "Deformable Registration Fine" );
	memorymeter.Stop( "Deformable Registration Fine" );
	}
	catch( itk::ExceptionObject & err )
	{
	std::cerr << "ExceptionObject caught !" << std::endl;
	std::cerr << err << std::endl;
	}
	// Software Guide : EndCodeSnippet

	std::cout << "Deformable Registration Fine Grid completed" << std::endl;
	std::cout << std::endl;


	// Report the time and memory taken by the registration
	chronometer.Report( std::cout );
	memorymeter.Report( std::cout );

	finalParameters = registration->GetLastTransformParameters();

	bsplineTransformFine->SetParameters( finalParameters );
	
//-----------------------------------------------------------------------------------------------------

	typedef itk::ResampleImageFilter<MovingImageType, FixedImageType>    ResampleFilterType;

	ResampleFilterType::Pointer resample = ResampleFilterType::New();

	resample->SetTransform(bsplineTransformFine);
	// resample->SetTransform(outputCompositeTransform);
	resample->SetInput(movingImageReader->GetOutput());

	resample->SetSize(fixedImage->GetLargestPossibleRegion().GetSize());
	resample->SetOutputOrigin(fixedImage->GetOrigin());
	resample->SetOutputSpacing(fixedImage->GetSpacing());
	resample->SetOutputDirection(fixedImage->GetDirection());

	// This value is set to zero in order to make easier to perform
	// regression testing in this example. However, for didactic
	// exercise it will be better to set it to a medium gray value
	// such as 100 or 128.
	resample->SetDefaultPixelValue(100);	// Changed from 0.

	typedef  signed short  OutputPixelType;

	typedef itk::Image< OutputPixelType, ImageDimension > OutputImageType;

	typedef itk::CastImageFilter<FixedImageType, OutputImageType> CastFilterType;

	typedef itk::ImageFileWriter< OutputImageType>  WriterType;


	WriterType::Pointer      writer = WriterType::New();
	CastFilterType::Pointer  caster = CastFilterType::New();

	writer->SetFileName(outputImageFile);

	caster->SetInput(resample->GetOutput());
	writer->SetInput(caster->GetOutput());

	std::cout << "Writing resampled moving image...";

	try
	{
		writer->Update();
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	std::cout << " Done!" << std::endl;


	typedef itk::SquaredDifferenceImageFilter<
		FixedImageType,
		FixedImageType,
		OutputImageType > DifferenceFilterType;

	DifferenceFilterType::Pointer difference = DifferenceFilterType::New();

	WriterType::Pointer writer2 = WriterType::New();
	writer2->SetInput(difference->GetOutput());

	// Compute the difference image between the
	// fixed and resampled moving image.
	difference->SetInput1(fixedImageReader->GetOutput());
	difference->SetInput2(resample->GetOutput());
	writer2->SetFileName(differenceImageFile);

	std::cout << "Writing difference image after registration...";

	try
	{
		writer2->Update();
	}
	catch (itk::ExceptionObject & err)
	{
		std::cerr << "ExceptionObject caught !" << std::endl;
		std::cerr << err << std::endl;
	}

	std::cout << " Done!" << std::endl;
}

