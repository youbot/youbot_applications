Diagnostics Tool
----------------

The Diagnostics Tool will perform an interactive self test to check the state of
the hardware components of the KUKA youBot. A summary as well as a logfile with
the test result will be generated.


Installation
------------

You must have installed the KUKA youBot API to be able to compile this application.

If not already done, check out the source code from GIT first to one of your folders:
$ git clone https://youbot@github.com/youbot/youbot_applications/diagnostics_tool.git

In order to compile, follow these steps:
$ cd <your_folder>/diagnostics_tool
$ mkdir build
$ cd build
$ cmake ..
$ make

The binaries will be generated in the folder ~/diagnostics_tool/bin.


Usage
-----

To start the application use:

$ cd <your_folder>/diagnostics_tool/bin
$ sudo ./youBot_DiagnosticsTool

The program will ask for the number of bases and arms, which is typically 1 for each
in case of a standard KUKA youBot system with one base and one arm. In the case one
motor has broken the program might ask you to move a joint manually. This is important
to distinguish between a broken motor or a broken controller.


License
-------

This software is published under a dual-license: GNU Lesser General Public
License LGPL 2.1 and BSD license. The dual-license implies that users of this
code may choose which terms they prefer.


Support
-------

Mail to users@youbot-community.org  
or go to http://lists.youbot-community.org/mailman/listinfo/users to 
subscribe to the mailing list
