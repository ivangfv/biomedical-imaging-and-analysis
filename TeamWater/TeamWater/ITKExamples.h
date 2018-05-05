#ifndef _ITKEXAMPLES_H
#define _ITKEXAMPLES_H

void DicomSeriesReadImageWrite2(const char* dicomDirectory, const char* outputFileName);
void ImageReadImageSeriesWrite(const char* inputFile, const char* outputPrefix, const char* outputExtension);
void BSplineRegistration(const char* fixedImageFile, const char* movingImageFile, const char* outputImageFile, const char* differenceImageFile);

#endif