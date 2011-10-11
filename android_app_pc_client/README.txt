-----------------------
YouBot.APK Installation
-----------------------
The common way to install an APK file (Android Package) is to use app installer that can be acquired in Android Market.
	- Go to Android Market
	- Download and install an app installer
	- Copy YouBot.APK to SD card
	- Run app installer. App installer will detect all APK file in SD card.
	- Install YouBot.APK


-----------------------
youBotAndroid Installation
-----------------------

youBotAndroid installation is similar with the installation of other KUKA youBot application such as Hello_World_demo or Joypad_application. 
After KUKA youBot application installation you will have a directory for KUKA youBot application
youBotAndroid installation require BlueZ development library and KUKA youBot API.

	- copy the folder "android" to KUKA youBot application directory
	- Add BlueZ development library (modification on CMakeList.txt or manually copy BlueZ library into folder "include" in KUKA youBot application directory)
	- on directory "android/build"
		$ cmake ..
		$ make

If the installation is successful, you will found "youBotAndroid" in directory "android/bin"
	- 

-----------------------
Running YouBot App
-----------------------
After YouBot.APK and youBotAndroid are installed, KUKA youBot can now be controlled from youBot App.
It is suggested that the KUKA youBot PC are already paired as bluetooth device with Android mobile device before running YouBot App.
	- Run youBotAndroid on KUKA youBot PC 
	- Run YouBot App on Android mobile device
	- Click "Select Device" from main menu
	- Select KUKA youBot PC. After selecting the device, YouBot app will return automatically to the main menu
	- Select controller mode (DRIVE mode or SHIFT mode)
	- Once the controller selected, YouBot App will automatically try to connect with the socket opened by youBotAndroid on PC.
	- Control KUKA youBot.