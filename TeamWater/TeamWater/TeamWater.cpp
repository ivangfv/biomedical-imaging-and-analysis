#include <iostream>
#include <string>
#include "ITKExamples.h"

/*
* User selects how they would like to process images from the main menu.
*/
int main(int argc, char *argv[]) {
	int mainMenuChoice = 0;
	int registrationChoice = 0;

	// Main Menu. 
	std::cout << "Welcome to Team Water Image Processor. What would you like to do?" << std::endl;
	std::cout << "1. Convert 2D DICOM image series to 3D image" << std::endl;
	std::cout << "2. Convert 3D image to PNG series" << std::endl;
	std::cout << "3. Registration of 3D images" << std::endl;
	std::cout << "4. Watershed Segmentation" << std::endl;
	std::cout << "5. Region-growing Segmentation" << std::endl;
	std::cout << std::endl;

	std::cout << "Enter the number corresponding to your choice: ";
	std::cin >> mainMenuChoice;
	std::cout << std::endl;

	// User chooses to convert a 2D DICOM image series to a 3D image
	if (mainMenuChoice == 1) {
		// outputImageFile must be .mhd format.
		std::string dicomDirectory, output3DImageName;

		std::cout << "You selected option 1" << std::endl;
		std::cout << "Please enter: dicomDirectory output3DImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> dicomDirectory >> output3DImageName;
		std::cout << std::endl << std::endl;

		std::cout << "Converting DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(dicomDirectory.c_str(), output3DImageName.c_str());
		std::cout << "Conversion complete" << std::endl;
	}

	// User chooses to convert a 3D image (mhd format) to a PNG series
	else if (mainMenuChoice == 2) {
		std::string input3DImageName, outputPNGImageName, pngExtension;

		std::cout << "You selected option 2" << std::endl;
		std::cout << "Please enter: input3DImageName.mhd outputPNGImageName png" << std::endl;
		std::cout << "Input: ";
		std::cin >> input3DImageName >> outputPNGImageName >> pngExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting 3D image to PNG series" << std::endl;
		ImageReadImageSeriesWrite(input3DImageName.c_str(), outputPNGImageName.c_str(), pngExtension.c_str());
		std::cout << "Conversion complete" << std::endl;
	}

	// User decides to perform registration. 
	else if (mainMenuChoice == 3) {
		std::string fixed3DImageName, moving3DImageName, outputImageName, differenceImageName;
		std::string outputPNGImageName, pngExtension;

		std::cout << "You selected option 3" << std::endl;

		std::cout << "Please enter: fixed3DImageName.mhd moving3DImageName.mhd outputImageName.mhd differenceImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> fixed3DImageName >> moving3DImageName >> outputImageName >> differenceImageName;
		std::cout << std::endl;

		// User chooses registration method.
		std::cout << "Select registration method" << std::endl;
		std::cout << "1. 3D Affine registration" << std::endl;
		std::cout << "2. 3D BSpline registration (WARNING: VERY SLOW)" << std::endl << std::endl;
		std::cout << "Input: ";
		std::cin >> registrationChoice;
		std::cout << std::endl << std::endl;

		if (registrationChoice == 1) {
			std::cout << "Performing 3D Affine registration" << std::endl;
			AffineRegistration(fixed3DImageName.c_str(), moving3DImageName.c_str(), outputImageName.c_str(), differenceImageName.c_str());
		}
		else if (registrationChoice == 2) {
			std::cout << "Performing 3D BSpline registration" << std::endl;
			BSplineRegistration2(fixed3DImageName.c_str(), moving3DImageName.c_str(), outputImageName.c_str(), differenceImageName.c_str());
		}
		else {
			// error message.
		}

		// Convert registration output image to PNG series.
		std::cout << "Converting registered image to PNG series" << std::endl;
		std::cout << "Please enter: outputPNGImageName pngExtension" << std::endl;
		std::cout << "Input: ";
		std::cin >> outputPNGImageName >> pngExtension;
		std::cout << std::endl << std::endl;

		ImageReadImageSeriesWrite(outputImageName.c_str(), outputPNGImageName.c_str(), pngExtension.c_str());
		std::cout << "Conversion complete" << std::endl;
	}

	// User decides to perform segmentation.
	// Program converts 3D image to PNG series.
	// Then, the user selects which PNG image they'd like to segment.
	else if (mainMenuChoice == 4) {

		// For Watershed segmentation function.
		std::string inputPNGImage, outputPNGImage, conductanceTerm, diffusionIterations;
		std::string lowerThreshold, outputScaleLevel, gradientMode;

		std::cout << "You selected option 4" << std::endl << std::endl;

		std::cout << "Preparing Watershed Segmentation." << std::endl;

		std::cout << "Please enter: inputPNGImage.png outputPNGImage.png conductanceTerm diffusionIterations lowerThreshold outputScaleLevel gradientMode(on/off)" << std::endl;
		std::cout << "Input: ";
		std::cin >> inputPNGImage >> outputPNGImage >> conductanceTerm >> diffusionIterations >>
			lowerThreshold >> outputScaleLevel >> gradientMode;
		std::cout << std::endl;

		std::cout << "Performing Watershed segmentation" << std::endl;
		WatershedSegmentation1(inputPNGImage.c_str(), outputPNGImage.c_str(), conductanceTerm.c_str(),
			diffusionIterations.c_str(), lowerThreshold.c_str(), outputScaleLevel.c_str(), gradientMode.c_str());

		std::cout << "Watershed segmentation complete" << std::endl;
	}

	else if (mainMenuChoice == 5) {
		std::string inputPNGImage, outputPNGImage, seedX, seedY, lowerThreshold, upperThreshold;

		std::cout << "You selected option 5" << std::endl << std::endl;
		std::cout << "Preparing region-growing segmentation." << std::endl;

		std::cout << "Please enter: inputPNGImage.png outputPNGImage.png seedX seedY lowerThreshold upperThreshold" << std::endl;
		std::cout << "Input: ";
		std::cin >> inputPNGImage >> outputPNGImage >> seedX >> seedY >> lowerThreshold >> upperThreshold;
		std::cout << std::endl << std::endl;

		std::cout << "Performing region-growing segmentation" << std::endl;
		NeighborhoodConnectedImageFilter(inputPNGImage.c_str(), outputPNGImage.c_str(), seedX.c_str(), 
										 seedY.c_str(), lowerThreshold.c_str(), upperThreshold.c_str());

		std::cout << "Region-growing segmentation complete" << std::endl;
	}

	return 0;
}


