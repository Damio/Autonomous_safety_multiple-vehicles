The one agent case moving obstacles

BEFORE RUNNING THIS CASE, COPY THE "main_script1" SCRIPT INTO THE ONE AGENT CASE FOLDER, so it can make use of the functions there.

Running this script displays the path wich a single vehicle takes to avoid multiple MOVING obstacles.

Before running the code running the code:

1) Check all the initial conditions to make sure they are in line with what you want(e.g obstacle radius...)
2) Check your time step to make sure it also fits.
3.) Also check the gains k_u & k_w and adjust as needed.
4.) Make sure you check the numberofObstacles variable to select how many obstacles you want.
5.) Make sure to change the name of the video stored in variable v, so it does not overwrite the previous one.

After you run the code:
1.) A plot window will pop up, this window allows you to click on any part of the window to create a point, the input requirements are as follows
	a. Start position
	b. Goal position
	c. All the obstacles, depending on the number specified for the numberOfObstacles variable.

While the code runs:

The code will beging to loop accordin to the specified timestep & call all the other function scripts in the folder as it runs in the process of updating the vehicle position.
You can manually end the running code by stopping the simulation at any time

The major difference between this case and the stationary obstacle case is the visualization of the result. In this case, the output is a video that is saved into the same folder as the rest of your code.


NOTE: Changes to the time step and gains k_u and k_w will directly affect the performance of this code