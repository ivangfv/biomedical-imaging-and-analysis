#ifndef _ITKEXAMPLES_H
#define _ITKEXAMPLES_H

void DicomSeriesReadImageWrite2(const char* dicomDirectory, const char* outputImageFile);
void ImageReadImageSeriesWrite(const char* inputFile, const char* outputPrefix, const char* outputExtension);
void BSplineRegistration(const char* fixedImageFile, const char* movingImageFile, const char* outputImageFile, const char* differenceImageFile);
void BSplineRegistration2(const char* fixedImageFile, const char* movingImageFile, const char* outputImageFile, const char* differenceImageFile);
void AffineRegistration(const char* fixedImageFile, const char* movingImageFile, const char* outputImageFile, const char* differenceImageFile);
void WatershedSegmentation1(const char* inputImageFile, const char* outputImageFile, const char* conductanceTerm, const char* diffusionIterations, 
	const char* lowerThreshold, const char* outputScaleLevel, const char* gradientMode);
void NeighborhoodConnectedImageFilter(const char* inputImageFile, const char* outputImageFile, const char* seedX,
	const char* seedY, const char* lowThreshold, const char* highThreshold);
#endif