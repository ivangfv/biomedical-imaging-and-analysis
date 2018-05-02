#include <iostream>

/*
* User selects how they would like to process images from the main menu.
*/
int main(int argc, char *argv[]) {
	int choice = 0;

	// Main Menu. 
	std::cout << "Welcome to Team Water Image Processor. What would you like to do?\n"
		<< "1. Image Registration\n"
		<< "2. Image Segmentation\n"
		<< "3. Both\n" << std::endl;

	std::cout << "Enter the number corresponding to your choice: ";
	std::cin >> choice;

	// User decides to perform registration. 
	if (choice == 1) {
		std::cout << "Performing registration" << std::endl;

		// Code for registration. Use ITK registration example.
	}

	// User decides to perform segmentation.
	else if (choice == 2) {
		std::cout << "Performing segmentation" << std::endl;

		// Code for segmentation. Use ITK segmentation example.

	}

	// User decides to perform both registration & segmentation. 
	else if (choice == 3) {
		std::cout << "Performing registration and segmentation." << std::endl;

		// Code for registration + segmentation. 
	}

	// User input is invalid. 
	else {
		// Error message.
	}

	return 0;
}


