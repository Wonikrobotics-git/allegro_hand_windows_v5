# allegro_hand_windows_v5

This application is lightweight control and communication software for the Allegro Hand (AH) V5 4Finger using Windows.

Peak Release/myAllegroHand*.exe:

 Allegro Hand control and CAN communication module. You can easily test your hand with this program.

**Recommend to read User's manual**
 
Programming Instructions
============ 

 1. Open up the solution, myAllegroHand.sln, in Visual Studio
 2. Right click the project 'myAllegroHand' in the Solution Explorer and click 'Properties'
 3. At the top of the 'Property Pages' window, set the Configuration to 'Peak Release' and check the properties below.
    <img width="896" alt="1" src="https://github.com/user-attachments/assets/aaa21316-c324-4c23-96c8-a309e0bb1b85">
    <img width="892" alt="2" src="https://github.com/user-attachments/assets/c63dcf9f-ae3e-4c5e-adfe-6ed8c13babfb">
    <img width="896" alt="3" src="https://github.com/user-attachments/assets/dc3119c7-e4ec-4fcd-bd1d-7c56a3c41842">

 4. Open myAllegroHand.cpp and, near the top, find comment '// USER HAND CONFIGURATION' and the constants below it

 5. bool RIGHT_HAND: Set to 'true' if using a right AH and 'false' if using left AH based on Serial number
 6. bool HAND_TYPE_A: Set to 'true' if using a non-Geared AH and 'false' if using Geared AH based on Serial number
 7. const int HAND_VERSION: For version 5.x, set to '5'.
 
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
