# biomedical-imaging-and-analysis
A hands-on group research project for developing a part of biomedical image analysis system using public libraries. We will design, impplement, and evaluate specific data analysis algorithms involving segmentation and registration of 3D medical images.

# Team Water
Nikita Bajracharya, Albert Du, Regine Manuel, and Ivan Villamar

# Instructions
This application requires an image viewer that can view .mhd images. We used ImageJ with Kang Li's
MetaImage reader plugin, which can be downloaded here: 
https://www.cs.cmu.edu/~kangli/code/MetaImage_Reader_Writer.html

Run application from Build/Debug/TeamWater.exe

Main menu will appear.

Welcome to Team Water Image Processor. What would you like to do?

  1. Convert 2D DICOM image series to 3D image
  2. Convert 3D image to PNG series
	3. Registration of 3D images
	4. Watershed Segmentation
	5. Region-growing Segmentation
  
Enter the number corresponding to your choice: 
  
# Example registration and segmentation of 2D DICOM series 
Each step requires running TeamWater.exe

Following this example will register a 2D DICOM image series and segment the brain. 

Fixed and moving image directories containing DICOM images should be placed in Build/Debug, 
the same folder as the TeamWater.exe option.

This example uses images from https://imaging.nci.nih.gov/ncia/login.jsf

Search head-neck-cetuximab-Demo CT scans 

Patient ID: 0522c0001 

Scans: Neck^HeadNeckPETCT 

Time period: Baseline and Baseline + 4 months

Suppose fixed image directory is called Fixed
and moving image directory is called Moving

Step 1: Run TeamWater.exe and select option 1 from main menu. 
        
        Input: Fixed fixed.mhd
        
        This produces a 3D image called fixed.mhd

Step 2: Repeat step 1 with moving DICOM series
        
        Input: Moving moving.mhd 
        
        This produces a 3D image called moving.mhd
        
Step 3: Run TeamWater.exe and select option 3 from main menu to perform registration on the 2 3D images. 

        Input: fixed.mhd moving.mhd output.mhd difference.mhd
        
        This produces a registration result image called output.mhd and a difference image difference.mhd. 
        After registration is complete, the program will convert output.mhd into a PNG.
        
        Input: output png
        
        This converts output.mhd into a png series with prefix output and png extension. For example, output000.png to outputnnn.png
        
Step 4: Run TeamWater.exe and select option 5 to perform region-growing segmentation

        Input: output106.png segmentation.png 260 265 0 25
        
        This performs region-growing segmentation on the 107th 2D slice of the registered image and produces segmentation.png
        The seed point for the region is (260, 265) with lower threshold = 0 and upper threshold = 25.
        
        The region starts at (260, 265) and checks if neighboring pixels fit the threshold. If they do, 
        the pixel is added to the region.
        
 Step 4 can be performed on any of the output images
        
