#include <iostream>
#include <string>
#include "ITKExamples.h"

/*
* User selects how they would like to process images from the main menu.
*/
int main(int argc, char *argv[]) {
	int choice = 0;

	// Main Menu. 
	std::cout << "Welcome to Team Water Image Processor. What would you like to do?" << std::endl;
	std::cout << "1. BSpline Registration" << std::endl;
	std::cout << "2. Watershed Segmentation" << std::endl;
	std::cout << "3. Both" << std::endl << std::endl;

	std::cout << "Enter the number corresponding to your choice: ";
	std::cin >> choice;
	std::cout << std::endl;

	// User decides to perform registration. 
	if (choice == 1) {
		std::cout << "Performing BSpline registration" << std::endl;

		// Code for registration. Prompt user to enter parameters. 
		// Used ITK registration example RegistrationITKv3/DeformableRegistration15.cxx.
		
	}

	// User decides to perform segmentation.
	else if (choice == 2) {
		std::cout << "Performing Watershed segmentation" << std::endl;

		// Code for segmentation. Prompt user to enter parameters.
		// Going to use ITK segmentation example WatershedSegmentation2.cxx.

	}

	/*
		User decides to perform both registration & segmentation on a series of DICOM images.

		Before performing registration, the baseline & moving DICOM series need to be converted to a 3D image.
		mhd is a suitable file format for 3D images. The 3D image is then converted to a PNG series so that 
		it can be viewed.

		Summary: series of 2D DICOM images -> 3D image -> series of PNG images.

		Registration input: 3D baseline image, 3D moving image, output image, difference image.
		During registration: USE MHD FORMAT FOR REGISTRATION OUTPUT IMAGE & DIFFERENCE IMAGE.
	*/
	else if (choice == 3) {
		/*
			Parameters for registration.

			Dicom Directory : name of DICOM directory
			3D image file   : name of 3D output image file (use mhd file format)
			output prefix   : name of image file
			output extension: file format (use png)

			RegOutputImageFile : result of registration (use mhd format)
			differenceImageFile: difference between fixed image & resampled moving image (use mhd format).
								 Very little in this image means the registration was done well.
		*/
		std::string baseDicomDirectory, base3DOutputImageFile, baseOutputPrefix, baseOutputExtension;
		std::string movingDicomDirectory, moving3DOutputImageFile, movingOutputPrefix, movingOutputExtension;
		std::string RegOutputImageFile, differenceImageFile;

		// Convert baseline images.
		std::cout << "Converting baseline images." << std::endl;
		std::cout << "Please enter: baseDicomDirectory base3DOutputImageFile baseOutputPrefix baseOutputExtension" << std::endl;
		std::cout << "Input: ";
		std::cin >> baseDicomDirectory >> base3DOutputImageFile >> baseOutputPrefix >> baseOutputExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting baseline 2D DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(baseDicomDirectory.c_str(), base3DOutputImageFile.c_str());

		std::cout << "Converting 3D image to PNG series" << std::endl << std::endl;
		ImageReadImageSeriesWrite(base3DOutputImageFile.c_str(), baseOutputPrefix.c_str(), baseOutputExtension.c_str());
		
		// Convert moving images.
		std::cout << "Converting moving images" << std::endl;
		std::cout << "Please enter: MovingDicomDirectory output3DMovingImageFile outputPrefix outputExtension" << std::endl;
		std::cout << "Input: ";
		std::cin >> movingDicomDirectory >> moving3DOutputImageFile >> movingOutputPrefix >> movingOutputExtension;
		std::cout << std::endl << std::endl;

		std::cout << "Converting moving 2D DICOM series to 3D image" << std::endl;
		DicomSeriesReadImageWrite2(movingDicomDirectory.c_str(), moving3DOutputImageFile.c_str());

		std::cout << "Converting 3D image to PNG series" << std::endl << std::endl;
		ImageReadImageSeriesWrite(moving3DOutputImageFile.c_str(), movingOutputPrefix.c_str(), movingOutputExtension.c_str());

		// Perform BSpline registration.
		std::cout << "Performing BSpline registration" << std::endl << std::endl;

		std::cout << "Please enter: RegOutputImageFile differenceImageFile" << std::endl;
		std::cout << "Input: ";
		std::cin >> RegOutputImageFile >> differenceImageFile;
		std::cout << std::endl;
		BSplineRegistration(base3DOutputImageFile.c_str(), moving3DOutputImageFile.c_str(), RegOutputImageFile.c_str(), differenceImageFile.c_str());

		// Perform Watershed segmentation. 

	}

	// User input is invalid. 
	else {
		// Error message.
	}

	return 0;
}


