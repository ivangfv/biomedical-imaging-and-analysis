#include <iostream>

/*
* User selects how they would like to process images from the main menu.
*/
int main(int argc, char *argv[]) {
	int choice = 0;

	// Main Menu. 
	std::cout << "Welcome to Team Water Image Processor. What would you like to do?\n"
		<< "1. BSpline Registration\n"
		<< "2. Watershed Segmentation\n"
		<< "3. Both\n" << std::endl;

	std::cout << "Enter the number corresponding to your choice: ";
	std::cin >> choice;

	// User decides to perform registration. 
	if (choice == 1) {
		std::cout << "Performing BSpline registration" << std::endl;

		// Code for registration. Prompt user to enter parameters. 
		// Use ITK registration example DeformableRegistration15.cxx.
	}

	// User decides to perform segmentation.
	else if (choice == 2) {
		std::cout << "Performing Watershed segmentation" << std::endl;

		// Code for segmentation. Prompt user to enter parameters.
		// Use ITK segmentation example WatershedSegmentation2.cxx.

	}

	// User decides to perform both registration & segmentation. 
	else if (choice == 3) {
		std::cout << "Performing registration and segmentation." << std::endl;

		// Code for registration + segmentation. Prompt user to enter parameters.
	}

	// User input is invalid. 
	else {
		// Error message.
	}

	return 0;
}


