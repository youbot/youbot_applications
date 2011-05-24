/******************************************************************************
* Copyright (c) 2011
* Locomotec
*
* Author:
* Özgür Sen
*
*
* This software is published under a dual-license: GNU Lesser General Public
* License LGPL 2.1 and BSD license. The dual-license implies that users of this
* code may choose which terms they prefer.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of Locomotec nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License LGPL as
* published by the Free Software Foundation, either version 2.1 of the
* License, or (at your option) any later version or the BSD license.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License LGPL and the BSD license for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License LGPL and BSD license along with this program.
*
******************************************************************************/

#include <signal.h>
#include <unistd.h>
#include <iostream>

#include <SDL/SDL.h>
#include <SDL/SDL_joystick.h>

#include "YouBotJoypad.hpp"


using namespace std;


bool END = false;

void sigintHandler(int signal)
{
	END = true;
	printf("End!\n\r");
}


//#######################
//Using a Joypad with SDL
//#######################
int main() {

//	signal(SIGINT, sigintHandler);

	//Initialising SDL with joystick
	SDL_Event event;

	if (SDL_Init( SDL_INIT_VIDEO | SDL_INIT_JOYSTICK ) == -1)
	{
		printf("Can't init SDL:  %s\n", SDL_GetError());
		exit(1);
	}
	atexit(SDL_Quit);

	SDL_EnableKeyRepeat (SDL_DEFAULT_REPEAT_DELAY, 1);


	//getting joystick/pad
	int noOfJoy = SDL_NumJoysticks ();

	SDL_Joystick * myJoypad = SDL_JoystickOpen(noOfJoy-1);

	if( myJoypad  )
	{
		printf(">>>> Joypad found!  <<<<\n");
		printf(">>>> Name of found Joypad: %s <<<<\n",SDL_JoystickName (noOfJoy-1) );
	}
	else
	{
		cout << ">>>> No Joypad found!  <<<<\n";
		exit(-1);
	}

	//get specification of the joystick/joypad
	int numberOfAxis	= SDL_JoystickNumAxes (myJoypad);
	int numberOfBalls	= SDL_JoystickNumBalls(myJoypad);
	int numberOfHat		= SDL_JoystickNumHats (myJoypad);
	int numberOfButton	= SDL_JoystickNumButtons(myJoypad);

	YouBotJoypad remoteController(numberOfAxis, numberOfBalls, numberOfHat, numberOfButton);
	remoteController.init( "youbot-joypad");

	cout << "Joypad ready for use." << endl;
	// Starting the event loop
	 while (!END)
	 {
		while (SDL_PollEvent(&event))
		{
			try
			{
				switch(event.type)
				{
					case  SDL_JOYAXISMOTION:
						remoteController.axisEvent(event.jaxis.axis, event.jaxis.value);
						break;

					case  SDL_JOYHATMOTION:
						remoteController.hatEvent(event.jhat.hat, event.jhat.value);
						break;

					case  SDL_JOYBUTTONDOWN:
						remoteController.buttonPressedEvent(event.jbutton.button);
						break;

					case SDL_JOYBUTTONUP:
						remoteController.buttonReleasedEvent(event.jbutton.button);
						break;
				 }
			}
			catch( exception& ex)
			{
				cout << "WHAT: " << ex.what() << endl;
			}

			usleep(100);
		}
	}

	return 0;
 }
