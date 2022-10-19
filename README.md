# Drilling_master_thesis
Public repository including the code (and some tips) of the program for my master thesis. 

## RaiSim installation
In order to install RaiSim, follow the official documentation available [here](https://raisim.com/sections/Installation.html).
It is strongly recommended to follow the tutorial available [here](https://www.youtube.com/watch?v=sfCR75Q58vI&t=167s&pp=ugMICgJpdBABGAE%3D).

## Import the program
Assume that **raisim_ws** is my workspace (it depends on the previous step of installation).
Assume that **test** is the folder including the whole program.
Assume that **a1** is the folder including the urdf and mesh file of the Robot A1.

In order to import the program into the workspace, copy and paste the **a1** folder in the window that will be opened. 
On the terminal, open the rsc folder:

```
cd raisim_ws/raisimLib
nautilus rsc
```

Then, copy and paste the **test** folder in raisimLib folder.
On the terminal, open the raisimLib folder:

```
cd raisim_ws
nautilus raisimLib
```

## Compile and build
In order to compile and build the program, we must modify the CMakeList file.

Once the raisimLib has been opened, on the terminal open the editor:

```
gedit CMakeLists.txt 
```

Modify the following if statement, adding the folder **test** as subdirectory: 

<code>if(RAISIM_EXAMPLE)
    #add_subdirectory(examples)
    add_subdirectory(test)
endif()</code>

In order to compile the program, we will use the *make* command:

```
cd raisim_ws/raisimLib/build/test
make
```

## Launch 
In order to launch the program, we must launch the RaiSimUnity GUI and, then, our executable. 

**Warning**: in the anymal_like.cpp file, there is an absolute path pointing at the location of the activation license file. Before to launch the program, check your own path and modify it in the code. No realtive path has been used since the activation license is machine-dependent, in other words it keeps track of my specific machine. 

### Launch RaiSimUnity
On the terminal:

```
cd raisim_ws/raisimLib/raisimUnity/linux
./raisimUnity.x86_64
```

**Warning**: before to launch the program, remember to flag the *automatic connection* tag, otherwise the TCP/IP server will not start spinning.

### Launch the program
It's time to launch our program:

```
cd raisim_ws/raisimLib/build/test
./anymals_like
```






