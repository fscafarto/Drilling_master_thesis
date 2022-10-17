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
On the temrinal, open the raisimLib folder:

```
cd raisim_ws
nautilus raisimLib


```

## Launch the program
In order to launch the program, we must modify the CMakeList file.

Once the raisimLib has been opened, on the terminal open the editor:

```
gedit CMakeLists.txt 
```

Modify the following if statement, adding the folder **test** as subdirectory: 

<code>if(RAISIM_EXAMPLE)
    #add_subdirectory(examples)
    add_subdirectory(test)
endif()</code>




