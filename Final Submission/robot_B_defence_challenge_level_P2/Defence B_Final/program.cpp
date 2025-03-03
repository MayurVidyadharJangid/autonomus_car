// Player 2 - Defence Robot A - Challenge
#include <cstdio>
#include <iostream>
#include <fstream>
#include "image_transfer.h"
#include <Windows.h>
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"
#include "defence_functions.h"
#include "update_simulation.h"

using namespace std; 

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

extern robot_system S1;

int main()
{
	double x0, y0, theta0, max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int n_robot, tvalue;
	double x_obs[50] = { 0.0 }, y_obs[50] = { 0.0 };
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode;
	int capture = 0;
	int v_mode;
	
	// number of obstacles
	const int N_obs = 2;
	
	// file names for all the obstacles
	// -- you must have the same number of names as obstacles
	// -- the names can be repeated though if the image is the same
	char obstacle_file[N_obs][S_MAX] = { 
		"obstacle_black.bmp", "obstacle_black.bmp"	
	};	
	
	// obstacle locations
	// -- you must set one location for each obstacle
	
	x_obs[0] = 250; // pixels
	y_obs[0] = 320; // pixels

	x_obs[1] = 180; // pixels
	y_obs[1] = 180; // pixels
	
	// TODO: it might be better to put this model initialization
	// section in a separate function
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	activate_simulation(width1,height1,
		x_obs,y_obs,N_obs,
		"robot_B.bmp","robot_A.bmp","background.bmp",
		obstacle_file,D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);			
		
	// open an output file if needed for testing or plotting
//	ofstream fout("sim1.txt");
//	fout << scientific;
	
	// set simulation mode
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 2;
	set_simulation_mode(mode);	// check for updates while using this function.
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 330;	
	theta0 = 0;
	set_robot_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)
	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
   
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;
	
	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

	// NOTE: for two player mode you shouldn't set the opponent inputs 

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser.
	
	// regular vision
	image rgb, rgb0, a, b, label;
	int height, width;

// creating variables for different color centroid and their position
	double ic_col[12+1];// x coordinates of centroids of different color
	double jc_col[12+1];// y coordinates of centroids of different color
	double xa[12+1];
	double ya[12+1];
	double x_obs1, x_obs2, y_obs1, y_obs2, x_obs3, y_obs3;
	double ig, jg, ir, jr;
	double io, jo, ib, jb;
	double i_nearest_obs=0,j_nearest_obs=0;
	static int n = 0;
	static int init = 0;
	static double prev_io, prev_jo;
	double dist_o1_red, dist_o2_red, dist_o3_red;

	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width  = 640;
	height = 480;

	rgb.type = RGB_IMAGE;
	rgb.width = width;
	rgb.height = height;
	
	a.type = 1;//RGB_IMAGE
	a.width = (int)width1;
	a.height = (int)height1;

	// allocate memory for the images
	allocate_image(rgb);
    allocate_image(a);
	
	
	join_player();

	// measure initial clock time
	tc0 = high_resolution_time(); 

	while(1) 
	{

//		/*update_background();
//		update_obstacles();		
		
		// simulates the robots and acquires the image from simulation
		

//		update_image(rgb);

//		save_rgb_image("output.bmp",rgb);
//		cout << "\noutput image saved to file\n";
//		getchar();*/
		acquire_image_sim(rgb);

		tc = high_resolution_time() - tc0;

		image_processing(rgb, a, xa, ya, ic_col, jc_col);


		x_obs1 = ic_col[0];//obstacle 1 x coordinate
		x_obs2 = ic_col[1];//obstacle 2 x coordinate
		x_obs3 = ic_col[2];//obstacle 3 x coordinate
		y_obs1 = jc_col[0];//obstacle 1 y coordinate
		y_obs2 = jc_col[1];//obstacle 2 y coordinate
		y_obs3 = jc_col[2];//obstacle 3 y coordinate
		/*
				ig = ic_col[11];//robot green i coordinate
				jg = jc_col[11];//robot green j coordinate
				ir = ic_col[9];//robot red i coordinate
				jr = jc_col[9];//robot red j coordinate

				io = ic_col[12];//robot orange i coordinate
				jo = jc_col[12];//robot orange j coordinate
				ib = ic_col[10];//robot blue i coordinate
				jb = jc_col[10];//robot blue j coordinate
		*/
		// inverting the variables for red green with blue orange
		ig = ic_col[12];//robot green i coordinate
		jg = jc_col[12];//robot green j coordinate
		ir = ic_col[10];//robot red i coordinate
		jr = jc_col[10];//robot red j coordinate

		io = ic_col[11];//robot orange i coordinate
		jo = jc_col[11];//robot orange j coordinate
		ib = ic_col[9];//robot blue i coordinate
		jb = jc_col[9];//robot blue j coordinate

		// *****************************DEFENCE BOT ALGORITHM *********************************

		// ASSUMPTIONS:
		// 1. Atleast 1 obstacle must be placed in a valid zone i.e. there should
		// be enough space left for the robot to pass without getting off screen
		// 2. If 2 or more obstacles are placed in the environment, there should be 
		// enough space between them to allow the robot to pass without hitting them.
		// 3. The program is designed for Number of obstacles ranging from 1 to 3. However, the 
		// algorithm might get disturbed for N = 3 obstacles in some extreme cases.


		// NOTES:
		// 1. init variable used in the code helps in restricting the main program to run in small 
		// individual functions. This helps in preventing the running of all the functions simultaneously 
		// (which messes up the code as other functions interfere in the performance of the code)
		// 2. Before starting the two-player mode, please set the N_obs, same inital conditions of the 
		// obstacles and the robots in both of the attack and defence program.
		// 3. As a safety measure, the simulation will pause/stop when the robot gets offscreen and an 
		// error message will be printed in that case.
		// 4. All the functions used in the code are programmed separately in other cpp files and a header
		// file to maintain good standards of coding. Only the function calls and some safety checks are 
		// included in this main file.

		//------------------Number of Obstacles=1---------------------
		if (N_obs == 1) {
			// going to nearest obstacle
			if (init == 0) {
				go_to_obstacle(ig, jg, ir, jr, x_obs1, y_obs1, pw_l, pw_r, init);
			}
			// defining the coordinates of obstacle 
			i_nearest_obs = ic_col[1], j_nearest_obs = jc_col[1];

			// aligning at 90 deg with obstacle so as to maintain tangential trajectory
			if (init == 1) {
				aligning(ig, jg, ir, jr, io, jo, ib, jb, i_nearest_obs, j_nearest_obs,
					pw_l, pw_r, init);
			}

			// stopping at 90 deg with obstacle so as to maintain tangential trajectory
			if ((init == 2) || (init == 3)) {
				stop90deg(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs,
					pw_l, pw_r, init);
			}

			// revolving around obstacle
			if ((init == 4) || (init == 5)) {
				round_about(ig, jg, ir, jr, io, jo, ib, jb, i_nearest_obs,
					j_nearest_obs, pw_l, pw_r, init);
			}

			// checking if bot doesnt go out of screen and change its rotation sense
			if (init == 6) {
				if (ir < 40 || ir>600 || jr < 40 || jr>440) {
					pw_l = 2000;
					pw_r = 2000;
					init = 2;
				}
			}
			if (init == 7) {
				if (ir < 40 || ir>600 || jr < 40 || jr>440) {
					pw_l = 1000;
					pw_r = 1000;
					init = 3;
				}
			}

			// evading from opponent
			if (init == 6) {
				pw_l = 1000;
				pw_r = 1000;
				stop_at_zero(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs, io, jo, ib, jb,
					pw_l, pw_r, init, prev_io, prev_jo);
			}
			if (init == 7) {
				pw_l = 2000;
				pw_r = 2000;
				stop_at_zero(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs, io, jo, ib, jb,
					pw_l, pw_r, init, prev_io, prev_jo);
			}

			// restarting the loop if the opponent moves
			if (init == 8) {
				restart(io, jo, init, prev_io, prev_jo);
			}
		}


		//------------------Number of Obstacles=2---------------------
		else if (N_obs == 2) {
			// going to nearest obstacle 
			if (init == 0) {
				go_to_obstacle(ig, jg, ir, jr, x_obs1, y_obs1, x_obs2, y_obs2, pw_l, pw_r, n, init);
			}
			// finding the coordinates of obstacle nearest to robot
			if (n == 1) i_nearest_obs = ic_col[1], j_nearest_obs = jc_col[1];
			if (n == 2) i_nearest_obs = ic_col[2], j_nearest_obs = jc_col[2];

			// setting the other obstacle's coordinates to prevent the bot from hitting it
			if (n == 1) calc_distance(ir, jr, x_obs2, y_obs2, dist_o2_red);
			if (n == 2) calc_distance(ir, jr, x_obs1, y_obs1, dist_o2_red);

			// aligning at 90 deg with obstacle so as to maintain tangential trajectory
			if (init == 1) {
				aligning(ig, jg, ir, jr, io, jo, ib, jb, i_nearest_obs, j_nearest_obs, pw_l, pw_r, init);
			}

			// stopping at 90 deg with obstacle so as to maintain tangential trajectory
			if ((init == 2) || (init == 3)) {
				stop90deg(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs, pw_l, pw_r, init);
			}

			// revolving around obstacle
			if ((init == 4) || (init == 5)) {
				round_about(ig, jg, ir, jr, io, jo, ib, jb, i_nearest_obs,
					j_nearest_obs, pw_l, pw_r, init);
			}

			// checking if bot doesnt go out of screen and doesnt hit other obstacle
			if (init == 6) {
				if ((dist_o2_red < 90) || (ir < 40 || ir>600 || jr < 40 || jr>440)) {
					pw_l = 2000;
					pw_r = 2000;
					init = 2;
				}
			}
			if (init == 7) {
				if ((dist_o2_red < 90) || (ir < 40 || ir>600 || jr < 40 || jr>440)) {
					pw_l = 1000;
					pw_r = 1000;
					init = 3;
				}
			}

			// evading from opponent
			if (init == 6) {
				pw_l = 1000;
				pw_r = 1000;
				stop_at_zero(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs, io, jo, ib, jb,
					pw_l, pw_r, init, prev_io, prev_jo);
			}
			if (init == 7) {
				pw_l = 2000;
				pw_r = 2000;
				stop_at_zero(ig, jg, ir, jr, i_nearest_obs, j_nearest_obs, io, jo, ib, jb,
					pw_l, pw_r, init, prev_io, prev_jo);
			}


			// restarting the loop if the opponent moves

			if (init == 8) {
				restart(io, jo, init, prev_io, prev_jo);
			}
		}

		//-------------------------Additional Safety Control----------------------------

		if (ig < 15 || ig>625 || jg < 15 || jg>465) {
			cout << "\nRobot is outside of workspace, please restart!";
			pause();
		}
		/*
			//---------------------------------------------------------------------
			//****** Keyboard control*****
			// Can be used to test for keyboard control as opponent

			pw_l_o = 1500;
			pw_r_o = 1500;
			laser_o = 0;

			if (KEY('X'))
			{
				laser_o = 1;
			}

			if (KEY(VK_UP))
			{
				pw_l_o = 1000;
				pw_r_o = 2000;
			}
			if (KEY(VK_DOWN))
			{
				pw_l_o = 2000;
				pw_r_o = 1000;
			}
			if (KEY(VK_LEFT))
			{
				pw_l_o = 2000;
				pw_r_o = 2000;
			}
			if (KEY(VK_RIGHT))
			{
				pw_l_o = 1000;
				pw_r_o = 1000;
			}

			//**********************
			*/
		// change the inputs to move the robot around
		
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l,pw_r,pw_laser,laser,max_speed);

		// NOTE: only one program / player can call view_image()
		
		// * v_mode is an optional argument for view_rgb_image(...)
		// - adjusting it might improve performance / reduce delays
		// -- see "image_transfer.h" for more details
//		v_mode = 1;
//		view_rgb_image(rgb,v_mode);

		// * I removed the Sleep / delay function call below to 
		// improve performance / reduce delays, especially with 
		// player 1 / player 2 scenarios on laptops
		// -- it seems laptops tend to go into low CPU mode
		// when Sleep is called, which slows down the simulation
		// more than the requested sleep time
//		Sleep(10); // 100 fps max
	}

	// free the image memory before the program completes
	free_image(rgb);
	free_image(a);
	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}

