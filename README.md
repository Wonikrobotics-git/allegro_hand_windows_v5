# allegro_hand_windows_v5

This application is lightweight control and communication software for the Allegro Hand (AH) using Windows.

PEAK Release/myAllegroHand*.exe:

 Allegro Hand control and CAN communication module. You can easily test your hand with this program. 
 
Programming Instructions
============ 

 1. Open up the solution, myAllegroHand.sln, in Visual Studio
 2. Right click the project 'myAllegroHand' in the Solution Explorer and click 'Properties'
 3. At the top of the 'Property Pages' window, set Configuration to 'PEAK Release'
 4. Navigate to Configuration Properties > Debugging and set the Working Directory to 'bin'
 5. Open myAllegroHand.cpp and, near the top, find comment '// USER HAND CONFIGURATION' and the constants below it

 6. bool RIGHT_HAND: Set to 'true' if using a right AH and 'false' if using left AH based on Serial number
 7. bool HAND_TYPE_A: Set to 'true' if using a non-Geared AH and 'false' if using Geared AH based on Serial number
 8. const int HAND_VERSION: For version 5.x, set to '5'.
 
You are now ready to compile, plug in and turn on your hand, and test the program.

Keyboard commands can be used to execute grasps and other joint configurations. 
See the instructions printed at the User manual or at the beginning of the application.


# Control more than one hand

1. Open up the solution, myAllegroHand.sln, in Visual Studio.
2. Specify PCAN_USBBUS`N`.
3. Debug and start the program.

=============================

# Allegro Hand Standalone Visual Studio Project and Source

This file contains a summary of what you will find in each of the files that make up your myAllegroHand application.



**myAllegroHand.vcproj:**

This is the main project file for VC++ projects generated using an Application Wizard. It contains information about the version of Visual C++ that generated the file, and information about the platforms, configurations, and project features selected with the Application Wizard.

	
	
**myAllegroHand.cpp:**

This is the main application source file.

	
	
**Other standard files:**

StdAfx.h, StdAfx.cpp

These files are used to build a precompiled header (PCH) file named myAllegroHand.pch and a precompiled types file named StdAfx.obj.

	
	
**Other notes:**

AppWizard uses "TODO:" comments to indicate parts of the source code you should add to or customize.

For more information regarding setting the project properties, etc., please see the included document Allegro_Hand_MSVS_Project_*.pdf.

**Note:** This document is currently provided only in Korean. All important setup information can be gathered deirectly from the screenshots.



**Questions?**

Please use AllegroHand official Forum : https://www.allegrohand.com/forum
