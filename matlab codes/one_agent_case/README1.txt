The one agent case

Running this script displays the path wich a single vehicle takes to avoid multiple obstacles.

Before running the code running the code:

1) Check all the initial conditions to make sure they are in line with what you want(e.g obstacle radius...)
2) Check your time step to make sure it also fits.
3.) Also check the gains k_u & k_w and adjust as needed.
4.) Make sure you check the numberofObstacles variable to select how many obstacles you want.

After you run the code:
1.) A plot window will pop up, this window allows you to click on any part of the window to create a point, the input requirements are as follows
	a. Start position
	b. Goal position
	c. All the obstacles, depending on the number specified for the numberOfObstacles variable.

While the code runs:

The code will beging to loop accordin to the specified timestep & call all the other function scripts in the folder as it runs in the process of updating the vehicle position.
You can manually end the running code by stopping the simulation at any time

Once the code has stopped running, you should have a graph displaying all your obstacles, along with the path taken by the vehicle to its current position.



NOTE: Changes to the time step and gains k_u and k_w will directly affect the performance of this code