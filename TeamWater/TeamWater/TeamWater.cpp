#include <iostream>
#include <string>
#include "ITKExamples.h"

/*
* User selects how they would like to process images from the main menu.
*/
int main(int argc, char *argv[]) {
	int mainMenuChoice = 0;
	int registrationChoice1 = 0;
	int registrationChoice2 = 0;

	// Main Menu. 
	std::cout << "Welcome to Team Water Image Processor. What would you like to do?" << std::endl;
	std::cout << "1. Convert 2D DICOM image series to 3D image" << std::endl;
	std::cout << "2. Convert 3D image to PNG series" << std::endl;
	std::cout << "3. Registration of 3D images" << std::endl;
	std::cout << "4. Watershed Segmentation" << std::endl;
 	std::cout << "5. Registration & segmentation of 2D DICOM image series" << std::endl << std::endl;

	std::cout << "Enter the number corresponding to your choice: ";
	std::cin >> mainMenuChoice;
	std::cout << std::endl;

	// User chooses to convert a 2D DICOM image series to a 3D image
	if (mainMenuChoice == 1) {
		// outputImageFile must be .mhd format.
		std::string dicomDirectory, outputImageName;

		std::cout << "You selected option 1" << std::endl;
		std::cout << "Please enter: dicomDirectory outputImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> dicomDirectory >> outputImageName;
		std::cout << std::endl << std::endl;

		std::cout << "Converting DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(dicomDirectory.c_str(), outputImageName.c_str());
	}

	// User chooses to convert a 3D image (mhd format) to a PNG series
	else if (mainMenuChoice == 2) {
		std::string inputImageName, outputPNGImageName, pngExtension;

		std::cout << "You selected option 2" << std::endl;
		std::cout << "Please enter: inputImageName.mhd outputPNGImageName png" << std::endl << std::endl;
		std::cout << "Input: ";
		std::cin >> inputImageName >> outputPNGImageName >> pngExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting 3D image to PNG series" << std::endl;
		ImageReadImageSeriesWrite(inputImageName.c_str(), outputPNGImageName.c_str(), pngExtension.c_str());
	}

	// User decides to perform registration. 
	else if (mainMenuChoice == 3) {
		std::string fixed3DImageName, moving3DImageName, outputImageName, differenceImageName;

		std::cout << "You selected option 3" << std::endl << std::endl;

		std::cout << "Please enter: fixed3DImageName.mhd moving3DImageName.mhd outputImageName.mhd differenceImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> fixed3DImageName >> moving3DImageName >> outputImageName >> differenceImageName;
		std::cout << std::endl;

		// User chooses registration method.
		std::cout << "Select registration method" << std::endl;
		std::cout << "1. 3D Affine registration" << std::endl;
		std::cout << "2. 3D BSpline registration (WARNING: VERY SLOW)" << std::endl << std::endl;
		std::cout << "Input: ";
		std::cin >> registrationChoice1;
		std::cout << std::endl << std::endl;

		if (registrationChoice1 == 1) {
			std::cout << "Performing 3D Affine registration" << std::endl;
			AffineRegistration(fixed3DImageName.c_str(), moving3DImageName.c_str(), outputImageName.c_str(), differenceImageName.c_str());
		}
		else if (registrationChoice1 == 2) {
			std::cout << "Performing 3D BSpline registration" << std::endl;
			BSplineRegistration2(fixed3DImageName.c_str(), moving3DImageName.c_str(), outputImageName.c_str(), differenceImageName.c_str());
		}
		else {
			// error message.
		}

	}

	// User decides to perform segmentation.
	else if (mainMenuChoice == 4) {
		std::cout << "Performing Watershed segmentation" << std::endl;

		// Code for segmentation. Prompt user to enter parameters.
		// Going to use ITK segmentation example WatershedSegmentation2.cxx.

	}

	/*
		User decides to perform both registration & segmentation on a series of DICOM images.

		Before performing registration, the fixed & moving DICOM series need to be converted to a 3D image.
		mhd is a suitable file format for 3D images. The 3D image is then converted to a PNG series so that 
		it can be viewed.

		Summary: series of 2D DICOM images -> 3D image -> series of PNG images.

		Registration input: 3D baseline image, 3D moving image, output image, difference image.
		During registration: USE MHD FORMAT FOR REGISTRATION OUTPUT IMAGE & DIFFERENCE IMAGE.
	*/
	else if (mainMenuChoice == 5) {
		/*
			Parameters for registration.

			Dicom Directory : name of DICOM directory
			3D image Name   : name of 3D output image file (use mhd file format)
			PNG Image Name  : name of PNG image file converted from 3D image
			PNGoutput extension: PNG file format (user enters png for this parameter)

			RegOutputImageName : name of registration output image (use mhd format). 
			differenceImageName: name of difference image between fixed image & resampled moving image (use mhd format).
		*/
		std::string fixedDicomDirectory, fixed3DOutputImageName, fixedPNGImageName, fixedPNGOutputExtension;
		std::string movingDicomDirectory, moving3DOutputImageName, movingPNGImageName, movingPNGOutputExtension;
		std::string RegOutputImageName, differenceImageName;

		// Convert fixed DICOM images.
		std::cout << "You selected option 5" << std::endl << std::endl;

		std::cout << "Preparing for registration. Need to convert DICOM images to 3D and PNG." << std::endl << std::endl;
		std::cout << "Please enter: fixedDicomDirectory fixed3DOutputImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> fixedDicomDirectory >> fixed3DOutputImageName;
		std::cout << std::endl << std::endl;

		std::cout << "Converting fixed 2D DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(fixedDicomDirectory.c_str(), fixed3DOutputImageName.c_str());

		std::cout << "Please enter: fixedPNGImageName png" << std::endl;
		std::cout << "Input: ";
		std::cin >> fixedPNGImageName >> fixedPNGOutputExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting fixed 3D image to PNG series" << std::endl << std::endl;
		ImageReadImageSeriesWrite(fixed3DOutputImageName.c_str(), fixedPNGImageName.c_str(), fixedPNGOutputExtension.c_str());
		
		// Convert moving images.
		std::cout << "Converting moving images." << std::endl;
		std::cout << "Please enter: movingDicomDirectory moving3DOutputImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> movingDicomDirectory >> moving3DOutputImageName;
		std::cout << std::endl << std::endl;

		std::cout << "Converting moving 2D DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(movingDicomDirectory.c_str(), moving3DOutputImageName.c_str());

		std::cout << "Please enter: movingPNGImageName png" << std::endl;
		std::cout << "Input: ";
		std::cin >> movingPNGImageName >> movingPNGOutputExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting moving 3D image to PNG series" << std::endl << std::endl;
		ImageReadImageSeriesWrite(moving3DOutputImageName.c_str(), movingPNGImageName.c_str(), movingPNGOutputExtension.c_str());

		std::cout << "Please enter: RegistrationOutputImageName.mhd differenceImageName.mhd" << std::endl;
		std::cout << "Input: ";
		std::cin >> RegOutputImageName >> differenceImageName;
		std::cout << std::endl << std::endl;

		// User chooses registration method.
		std::cout << "Select registration method" << std::endl;
		std::cout << "1. 3D Affine registration" << std::endl;
		std::cout << "2. 3D BSpline registration (WARNING: VERY SLOW)" << std::endl << std::endl;
		std::cout << "Input: ";
		std::cin >> registrationChoice2;
		std::cout << std::endl << std::endl;

		if (registrationChoice2 == 1) {
			std::cout << "Performing 3D Affine registration" << std::endl;
			AffineRegistration(fixed3DOutputImageName.c_str(), moving3DOutputImageName.c_str(), RegOutputImageName.c_str(), differenceImageName.c_str());

		}
		else if (registrationChoice2 == 2) {
			std::cout << "Performing 3D BSpline registration" << std::endl;
			BSplineRegistration2(fixed3DOutputImageName.c_str(), moving3DOutputImageName.c_str(), RegOutputImageName.c_str(), differenceImageName.c_str());
		}
		else {
			// Error message.
		}

		// Perform Watershed segmentation. 
		std::cout << "Preparing for segmentation" << std::endl;
	}

	// User input is invalid. 
	else {
		// Error message.
	}

	return 0;
}


