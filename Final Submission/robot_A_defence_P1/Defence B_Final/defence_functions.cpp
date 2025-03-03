#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include <cmath>
#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "defence_functions.h"

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

void image_processing(image& rgb, image& a, double xi[], double yi[], double xo[], double yo[])
{
	image ai, bi, labeli, rgbi, rgbi2;
	ibyte* p0, * p;
	int k, R, G, B;
	int j, i;//iterators

	ai.type = GREY_IMAGE;
	ai.width = rgb.width;
	ai.height = rgb.height;
	allocate_image(ai);

	bi.type = GREY_IMAGE;
	bi.width = rgb.width;
	bi.height = rgb.height;
	allocate_image(bi);

	labeli.type = LABEL_IMAGE;
	labeli.width = rgb.width;
	labeli.height = rgb.height;
	allocate_image(labeli);

	rgbi.type = RGB_IMAGE;
	rgbi.width = rgb.width;
	rgbi.height = rgb.height;
	allocate_image(rgbi);

	rgbi2.type = RGB_IMAGE;
	rgbi2.width = rgb.width;
	rgbi2.height = rgb.height;
	allocate_image(rgbi2);


	copy(rgb, rgbi);

	p0 = rgbi.pdata;// Setting the pointer to the (0,0) position.
	p = rgbi.pdata;

	for (j = 0; j < rgbi.height; j++) { // j coord

		for (i = 0; i < rgbi.width; i++) { // i coord

			k = i + (rgbi.width * j);
			p = p0 + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *p;
			G = *(p + 1);
			R = *(p + 2);

			// for background
			if ((B > 160 && B < 225) && (G > 160 && G < 225) && (R > 150 && R < 215))
			{
				// make background as black
				//B = 0;
				//G = 0;
				//R = 0;

				*p = 0;
				*(p + 1) = 0;
				*(p + 2) = 0;
			}
			else {
				// make every object in the image as white(except background)
				//B = 255;
				//G = 255;
				//R = 255;

				// write the pixel values
				*p = 255;
				*(p + 1) = 255;
				*(p + 2) = 255;

			}
		}
	}


	copy(rgbi, bi);// taking the binary image of RGB type to GREY type
	// we need to do some erosion so that the binaryimage is more clearer and crisp,
	// and there is finaly only six clear white circles
	for (j = 0; j <= 10; j++) {
		erode(bi, ai);
		copy(ai, bi);
	}

	//----------------------------------------------------
	int nlabels;
	label_image(bi, labeli, nlabels);
	//cout << "\nnlabels : " << nlabels;

	// if want sample of binary image after threshold
	//copy(bi, rgbi);
	//save_rgb_image("binary_s.bmp", rgbi);


	for (j = 0; j < 13; j++);
	{// this is for ,if one of the robots get out of the image space
		xi[j] = xo[j] = 0.0;
		yi[j] = yo[j] = 0.0;
	}


	copy(rgb, ai);// creating new grey copy for checking colour not for size

	copy(rgb, rgbi);
	copy(rgb, rgbi2);// creating new  copy for checking  size

	centroid_rgb_image(rgbi2, xo[9], yo[9], 1);
	centroid_rgb_image(rgbi2, xo[10], yo[10], 2);
	centroid_rgb_image(rgbi2, xo[11], yo[11], 3);


	for (j = 1; j <= nlabels; j++)
	{
		centroid(bi, labeli, j, xi[j], yi[j]);
		//draw_point(ai,(int)xi[j],(int)yi[j],125);
		//draw_point_rgb(rgbi,(int)xi[j],(int)yi[j],125,35,35);
	}

	size_check(bi, rgbi, xi, yi, xo, yo);
	color_check(ai, rgbi, rgbi2, xo, yo);

	//sending back to the main program 
	copy(ai, a);
	copy(rgbi, rgb);


	free_image(rgbi);
	free_image(ai);
	free_image(bi);
	free_image(labeli);
	free_image(rgbi2);

}

// NOTE:
// Please uncomment the draw_point_rgb() function to see the points drawn on the centroids in simualtion

int centroid_rgb_image(image& rgb, double& ic, double& jc, int value)
{
	// value = 1 -- RED circle   R > B && R > G
	// value = 2 -- BLUE circle  B > R && B > G
	// value = 3 -- GREEN circle G > B && G > R
	// value = 4 -- ORANGE circle R > G && R > B

	// highlighting pixel is also redundant so it is commented out

	ibyte* pa_rgb, R, G, B;
	i4byte i, j, k, width, height, size;
	double mi, mj, m;

	if (rgb.type != RGB_IMAGE)
	{
		cout << ("\nError in Image loading , image is not RGB");
		return 1;
	}

	width = rgb.width;
	height = rgb.height;

	size = (i4byte)width * height * 3;
	pa_rgb = rgb.pdata; // initiate pointer
	mi = mj = m = 0.0;

	for (j = 0; j < height; j++) {// vertical
		for (i = 0; i < width; i++) {// horiozntal
			k = (width * j) + i;
			B = *(pa_rgb + 3 * k);
			G = *(pa_rgb + 3 * k + 1);
			R = *(pa_rgb + 3 * k + 2);

			if (value == 1) {//finding red circle
				if ((G > B) && (R > B) && (R >= 170) && (G <= 93) && (B <= 81)) {// && (G > B)(R > B) && 
					B = 0;
					R = 255;
					G = 0;

					*(pa_rgb + 3 * k) = B;
					*(pa_rgb + 3 * k + 1) = G;
					*(pa_rgb + 3 * k + 2) = R;

					m += R;
					mi += (double)i * R;
					mj += (double)j * R;
				}
			}

			if (value == 2) {//finding blue circle
				if ((B > R) && (B > G) && (B >= 220) && (G <= 165) && (R <= 55)) {
					//if ((B > R) && (B > G) && (B >= 213) && (G <= 161) && (R <= 68)) {
					B = 255;
					R = 0;
					G = 0;
					*(pa_rgb + 3 * k) = B;
					*(pa_rgb + 3 * k + 1) = G;
					*(pa_rgb + 3 * k + 2) = R;

					m += B;
					mi += (double)i * B;
					mj += (double)j * B;
				}
			}

			if (value == 3) {//finding green circle
				if ((G >= 176) && (R <= 78) && (R != 0) && (B <= 135) && (B != 0) && (G > B) && (G > R)) {
					//(G > B) && (G > R) && 
					B = 0;
					R = 0;
					G = 255;
					*(pa_rgb + 3 * k) = B;
					*(pa_rgb + 3 * k + 1) = G;
					*(pa_rgb + 3 * k + 2) = R;

					m += G;
					mi += (double)i * G;
					mj += (double)j * G;
				}
			}

			if (value == 4) {//finding orange circle
				if ((R > 250) && (G < 170 && G > 195) && (B > 110 && B < 135)) {
					B = 129;
					R = 254;
					G = 190;
					*(pa_rgb + 3 * k) = B;
					*(pa_rgb + 3 * k + 1) = G;
					*(pa_rgb + 3 * k + 2) = R;

					m += (double)R * G * B;
					mi += (double)i * R * G * B;
					mj += (double)j * R * G * B;
				}
			}

		}
	}

	double esp = 1.0e-6;
	ic = (double)mi / (m + esp);
	jc = (double)mj / (m + esp);

	return 0;
}


void color_check(image& ai, image& rgbi, image& rgbi2, double xo[], double yo[])
{
	image rgbc;
	rgbc.width = rgbi.width;
	rgbc.height = rgbi.height;
	rgbc.type = RGB_IMAGE;
	allocate_image(rgbc);


	ibyte* p_bc, * p_grc;
	int R, B, G, rho;
	int i, k;//iterator

	p_bc = rgbi2.pdata;
	p_grc = ai.pdata;

	//  7 -- RED circle  
	//  8 -- BLUE circle  
	//  9 -- GREEN circle 
	//  10 -- ORANGE circle 

	for (i = 5; i <= 8; i++)
	{
		k = ((int)yo[i] * rgbc.width) + (int)xo[i];
		rho = p_grc[k];
		B = p_bc[3 * k];
		G = p_bc[3 * k + 1];
		R = p_bc[3 * k + 2];

		//red
		if ((R == 255) && (G == 0) && (B == 0)) {
			//draw_point_rgb(rgbi, (int)xo[i], (int)yo[i], 150, 50, 54);
			xo[9] = xo[i];
			yo[9] = yo[i];
		}

		//green
		if ((G == 255) && (B == 0) && (R == 0)) {//&& (R > 60) && (R < 80) && (B > 120) && (B < 140))(G > R)&& (G > B)&& {
			//draw_point_rgb(rgbi, (int)xo[i], (int)yo[i], 50, 150, 54);
			xo[11] = xo[i];
			yo[11] = yo[i];
		}
		// blue
		if ((B == 255) && (R == 0) && (G == 0)) {//|| (G == 0) && (G <= 161) && (R <= 68))&& (B > R) && (B > G) &&{
			//draw_point_rgb(rgbi, (int)xo[i], (int)yo[i], 54, 50, 150);
			xo[10] = xo[i];
			yo[10] = yo[i];
		}

		//orange
		if ((rho >= 200) && (rho <= 202)) {
			//draw_point_rgb(rgbi, (int)xo[i], (int)yo[i], 195, 0, 0);
			xo[12] = xo[i];
			yo[12] = yo[i];
		}
	}

	free_image(rgbc);
}

void size_check(image& bi, image& rgbi, double xin[], double yin[], double xo[], double yo[])
// this function will help us to determine object's centroid based on the relative size
// any object whose size is greater than equal to 61 pixels in width and height 
{
	image rgbs;
	rgbs.width = rgbi.width;
	rgbs.height = rgbi.height;
	rgbs.type = RGB_IMAGE;
	allocate_image(rgbs);

	ibyte* p, * pr;
	int rh_c, rh_xr, rh_xl, rh_yu, rh_yd;
	int i, j1, j2;//iterator
	int k;
	p = bi.pdata;
	pr = rgbi.pdata;
	j1 = 1;
	j2 = 5;

	for (i = 1; i <= 10; i++)
	{
		k = (int)yin[i] * rgbi.width + (int)xin[i];
		if (k > 0)
		{
			rh_c = p[k];
			rh_xl = p[k - 22];
			rh_xr = p[k + 22];
			rh_yu = p[k + (22 * rgbi.width)];
			rh_yd = p[k - (22 * rgbi.width)];
			//cout << rh_c <<"," << rh_xl << "," << rh_xr << ","
			// << rh_yu << "," << rh_yd << "\n";

			//using binary image to check
			if (rh_c > 0 && rh_xr > 0 && rh_xl > 0 && rh_yu > 0 && rh_yd > 0)
			{
				//draw_point_rgb(rgbi, (int)xin[i], (int)yin[i], 255, 255, 255);
				xo[j1] = xin[i];
				yo[j1] = yin[i];
				j1++;
			}

			else
			{
				xo[j2] = xin[i];
				yo[j2] = yin[i];
				j2++;
			}
		}
	}
	// NOTE : remember the obstacle are numbered 1, starting from (0,0) 
	//lef bottom corner 
	
	free_image(rgbs);
}

//----------------------------Common Functions-----------------------------

void calc_distance(double x_1, double y_1, double x_2, double y_2, double& dist_xy) {

	dist_xy = sqrt(pow(x_1 - x_2, 2) + pow(y_1 - y_2, 2));

}

void calc_angle(double x_1, double y_1, double x_2, double y_2, double& theta)
{
	double dy, dx;
	dx = x_2 - x_1;
	dy = y_2 - y_1;

	//finding theta
	theta = atan2(dy, dx) * 180 / PI;
	if (theta < 0) theta += 360;

}

void aligning(double ig, double jg, double ir, double jr, double io, double jo, double ib,
	double jb, double i_nearest_obs, double j_nearest_obs, int& pw_l, int& pw_r, int& init)
{
	//cout << "\n align started";
	double i_gr_centre, j_gr_centre, i_ob_centre, j_ob_centre;
	double theta_robot, theta_bots;

	// average of both green and red centroids
	i_gr_centre = ir + (ig - ir) / 2.0;
	j_gr_centre = jr + (jg - jr) / 2.0;

	// average of both orange and blue centroids
	i_ob_centre = ib + (io - ib) / 2.0;
	j_ob_centre = jb + (jo - jb) / 2.0;

	calc_angle(ir, jr, ig, jg, theta_robot); // angle of robot

	// angle between the centres of both bots
	calc_angle(i_gr_centre, j_gr_centre, i_ob_centre, j_ob_centre, theta_bots);


	// checking if the opponent robot is towards the left side of our robot's pointing 
	// direction, then move right(clockwise), else move left(anti-clockwise)
	if ((theta_robot > 180) && (theta_robot < 360))
	{

		if ((theta_bots > theta_robot - 180) && (theta_bots < theta_robot)) {

			pw_l = 2000;//moving anti-clockwise
			pw_r = 2000;
			init = 2;

		}

		else {

			pw_l = 1000;//moving clockwise
			pw_r = 1000;
			init = 3;

		}
	}

	else
	{
		if (((theta_bots > 0) && (theta_bots < theta_robot)) ||
			((theta_bots > 180 + theta_robot) && (theta_bots < 360))) {
			pw_l = 2000;
			pw_r = 2000;
			init = 2;
		}
		else {
			pw_l = 1000;
			pw_r = 1000;
			init = 3;
		}
	}
}

void stop90deg(double ig, double jg, double ir, double jr, double i_nearest_obs,
	double j_nearest_obs, int& pw_l, int& pw_r, int& init)
{
	//cout << "\n stop at ninety started";
	double i_gr_centre, j_gr_centre;
	double theta_robot, theta_gr_obs;
	double diff;

	// average of both green and red centroids
	i_gr_centre = ir + (ig - ir) / 2.0;
	j_gr_centre = jr + (jg - jr) / 2.0;

	calc_angle(ir, jr, ig, jg, theta_robot); // angle of robot

	calc_angle(i_nearest_obs, j_nearest_obs, i_gr_centre, j_gr_centre, theta_gr_obs);

	theta_gr_obs = (int)theta_gr_obs;
	theta_robot = (int)theta_robot;
	diff = abs(theta_robot - theta_gr_obs);
	diff = (int)diff;

	// calculating slopes
	if (init == 2) {

		// additional safety check to prevent robot going out of image
		if (ir < 40 || ir>600 || jr < 40 || jr>440)
		{
			pw_l = 1500;
			pw_r = 1500;
			init = 4; // flag for anti-clockwise stop
		}
		// stopping at approx range of 90/270 degrees
		if (((diff < 90.0 - 10) && (diff > 90.0 + 10)) || ((diff < 270.0 - 10) && (diff < 270.0 + 10))) {
			pw_l = 1500;
			pw_r = 1500;
			init = 4; // flag for anti-clockwise stop
		}
	}
	else if (init == 3) {

		// additional safety check to prevent robot going out of image
		if (ir < 40 || ir>600 || jr < 40 || jr>440)
		{
			pw_l = 1500;
			pw_r = 1500;
			init = 5; // flag for clockwise stop
		}
		// stopping at approx range of 90/270 degrees
		if (((diff > 90.0 - 10) && (diff < 90.0 + 10)) || ((diff > 270.0 - 10) && (diff < 270.0 + 10))) {
			pw_l = 1500;
			pw_r = 1500;
			init = 5; // flag for clockwise stop
		}
	}
}

void round_about(double ig, double jg, double ir, double jr, double io, double jo, double ib,
	double jb, double i_nearest_obs, double j_nearest_obs, int& pw_l, int& pw_r, int& init)
{
	//cout << "\nround_about started";

	double i_gr_centre, j_gr_centre;
	double theta_robot_rg, theta_bot_obs, radius_revolute, theta_o_obs, radius_red, radius_green;
	int inclined;

	// Calculating robot mid point
	i_gr_centre = (ir + (ig - ir) / 2.0);
	j_gr_centre = (jr + (jg - jr) / 2.0);

	calc_distance(i_gr_centre, j_gr_centre, i_nearest_obs, j_nearest_obs, radius_revolute);
	calc_distance(ir, jr, i_nearest_obs, j_nearest_obs, radius_red);
	calc_distance(ig, jg, i_nearest_obs, j_nearest_obs, radius_green);

	calc_angle(ir, jr, ig, jg, theta_robot_rg);
	calc_angle(i_gr_centre, j_gr_centre, i_nearest_obs, j_nearest_obs, theta_bot_obs);
	calc_angle(io, jo, i_nearest_obs, j_nearest_obs, theta_o_obs);

	// setting angle to be inclined at

	inclined = (int)(theta_robot_rg - theta_bot_obs);
	inclined = (abs)(inclined);

	// setting parameters and starting revolution of robot around the obstacle

	if (((radius_revolute < 150) && (radius_revolute > 90)) ||
		(radius_red > 90) || (radius_green > 90)) {
		if (init == 4) {

			if (inclined > 180) {
				inclined = 360 - inclined;
			}

			if ((inclined >= 30) && (inclined <= 65)) {
				pw_l = 1000;//move forward
				pw_r = 2000;

			}
			else if ((inclined <= 29) && (inclined >= 0)) {
				pw_l = 2000;//anti-clockwise
				pw_r = 2000;

			}
			else if ((inclined <= 100) && (inclined >= 66)) {
				pw_l = 1000;//clockwise
				pw_r = 1000;

			}
			else if ((inclined >= 100) && (inclined <= 120)) {
				pw_l = 1000;//clockwise
				pw_r = 1000;

			}
			else if ((inclined <= 160) && (inclined >= 121)) {
				pw_l = 1000;//move forward
				pw_r = 2000;

			}
			else if ((inclined <= 180) && (inclined >= 161)) {
				pw_l = 2000;//anti-clockwise
				pw_r = 2000;

			}


			// checking for opponent laser and stopping

			if (theta_o_obs > 0 && theta_o_obs < 180) {
				//cout << "\nround about stop if conditions";
				if ((theta_bot_obs > theta_o_obs + 180 - 20) && (theta_bot_obs < theta_o_obs + 180 + 20))
				{
					pw_l = 1500;
					pw_r = 1500;
					init = 6;
				}
			}
			else {
				//cout << "\nround about stop else conditions";
				if ((theta_bot_obs > theta_o_obs - 180 - 20) && (theta_bot_obs < theta_o_obs - 180 + 20))
				{
					pw_l = 1500;
					pw_r = 1500;
					init = 6;
				}
			}
		}

		else if (init == 5)
		{

			if (inclined > 180) {
				inclined = 360 - inclined;
			}

			if ((inclined >= 30) && (inclined <= 65)) {
				pw_l = 1000;//move forward
				pw_r = 2000;

			}
			else if ((inclined <= 29) && (inclined >= 0)) {
				pw_l = 1000;//clockwise
				pw_r = 1000;

			}
			else if ((inclined <= 100) && (inclined >= 66)) {
				pw_l = 2000;//anti-clockwise
				pw_r = 2000;

			}
			else if ((inclined >= 101) && (inclined <= 120)) {
				pw_l = 2000;//anti-clockwise
				pw_r = 2000;

			}
			else if ((inclined <= 150) && (inclined >= 121)) {
				pw_l = 1000;//move forward
				pw_r = 2000;

			}
			else if ((inclined <= 180) && (inclined >= 151)) {
				pw_l = 1000;//clockwise
				pw_r = 1000;

			}

			// checking for opponent laser and stopping

			if (theta_o_obs > 0 && theta_o_obs < 180) {
				//cout << "\nround about stop if conditions";
				if ((theta_bot_obs > theta_o_obs + 180 - 20) && (theta_bot_obs < theta_o_obs + 180 + 20))
				{
					pw_l = 1500;
					pw_r = 1500;
					init = 7;
				}
			}

			else {
				//cout << "\nround about stop else conditions";
				if ((theta_bot_obs > theta_o_obs - 180 - 20) && (theta_bot_obs < theta_o_obs - 180 + 20))
				{
					pw_l = 1500;
					pw_r = 1500;
					init = 7;
				}
			}
		}
	}
	else {
		cout << "\n bot out of boundary";
		pause();
	}
}

void stop_at_zero(double ig, double jg, double ir, double jr, double i_nearest_obs, double j_nearest_obs,
	double io, double jo, double ib, double jb, int& pw_l, int& pw_r, int& init, double& prev_io, double& prev_jo)
{
	//cout<< "\n stop at zero started";
	double i_gr_centre, j_gr_centre;
	double theta_robot, theta_g_obs;

	// average of both green and red centroids
	i_gr_centre = ir + (ig - ir) / 2.0;
	j_gr_centre = jr + (jg - jr) / 2.0;

	calc_angle(ir, jr, ig, jg, theta_robot); // angle of robot
	calc_angle(ig, jg, i_nearest_obs, j_nearest_obs, theta_g_obs); // angle between robot head and obstacle centroid

	theta_g_obs = (int)theta_g_obs;
	theta_robot = (int)theta_robot;

	// providing a range to stop at 0 degrees
	if ((theta_robot > theta_g_obs - 5) && (theta_robot < theta_g_obs + 5)) {
		pw_l = 1500;
		pw_r = 1500;
		prev_io = io;
		prev_jo = jo;
		init = 8;
	}
}

void restart(double io, double jo, int& init, double& prev_io, double& prev_jo)
{
	// restarting the loop if the opponent moves by 10 pixels in any direction
	if (!((io > prev_io - 10) && (io < prev_io + 10) && (jo > prev_jo - 10) && (jo < prev_jo + 10))) {
		init = 1;
	}
}


//-----------------------Obstacles number == 1-----------------------------
void go_to_obstacle(double ig, double jg, double ir, double jr, double x_obs1, double y_obs1,
	int& pw_l, int& pw_r, int& init)
{
	//cout << "\ngo to obstacle";
	double theta_robot, theta_obs_1;
	double dist_o1_g_bot;
	double d_theta_o1;
	int safe_dist_o1;

	calc_distance(ig, jg, x_obs1, y_obs1, dist_o1_g_bot); //distance between robot head and obstacle 1

	calc_angle(ir, jr, ig, jg, theta_robot); // angle of robot
	calc_angle(ig, jg, x_obs1, y_obs1, theta_obs_1);//angle between robot head and obstacle 1

	d_theta_o1 = theta_robot - theta_obs_1;//angle difference between robot and obs 1

	// providing factor of safety to avoid collision with obstacle
	safe_dist_o1 = (int)dist_o1_g_bot - 100;

	// obstacle is within range to allow bot's width to pass
	if (x_obs1 > 140 && x_obs1 < 500 && y_obs1 > 140 && y_obs1 < 340)
	{
		if (d_theta_o1 < 0) {
			pw_l = 2000; //moving anti-clockwise
			pw_r = 2000;
		}
		else {
			pw_l = 1000; //moving clockwise
			pw_r = 1000;
		}
		if ((d_theta_o1 > -5) && (d_theta_o1 < 5)) {
			pw_l = 1000;//move forward
			pw_r = 2000;
		}
		if (safe_dist_o1 <= 0) {
			pw_l = 1500;//stop
			pw_r = 1500;
			init = 1;
		}
	}

	else {// printing out error message of obstacle's inital position
		cout << "Obstacle is not well placed, Robot can't revolve without getting off screen";
		pause();
	}
}


//-----------------------Obstacles number == 2-----------------------------
void go_to_obstacle(double ig, double jg, double ir, double jr, double x_obs1, double y_obs1,
	double x_obs2, double y_obs2, int& pw_l, int& pw_r, int& n, int& init)
{
	//cout << "\ngo to obstacle";
	double theta_robot, theta_obs_1, theta_obs_2;
	double dist_o1_g_bot, dist_o2_g_bot;
	double d_theta_o1, d_theta_o2;
	int safe_dist_o1, safe_dist_o2;


	calc_distance(ig, jg, x_obs1, y_obs1, dist_o1_g_bot); //distance between robot head and obstacle 1
	calc_distance(ig, jg, x_obs2, y_obs2, dist_o2_g_bot); //distance between robot head and obstacle 2

	calc_angle(ir, jr, ig, jg, theta_robot); // angle of robot
	calc_angle(ig, jg, x_obs1, y_obs1, theta_obs_1);//angle between robot head and obstacle 1
	calc_angle(ig, jg, x_obs2, y_obs2, theta_obs_2);//angle between robot head and obstacle 2

	d_theta_o1 = theta_robot - theta_obs_1;//angle difference between robot and obs 1
	d_theta_o2 = theta_robot - theta_obs_2;//angle difference between robot and obs 2


	// providing factor of safety to avoid collision with obstacle
	safe_dist_o1 = (int)dist_o1_g_bot - 100;
	safe_dist_o2 = (int)dist_o2_g_bot - 100;

	// both obstacles are within range to allow bot's width to pass
	if (x_obs1 > 140 && x_obs1 < 500 && x_obs2 > 140 && x_obs2 < 500 &&
		y_obs1 > 140 && y_obs1 < 340 && y_obs2 > 140 && y_obs2 < 340)
	{
		if (dist_o1_g_bot < dist_o2_g_bot) {
			// move towards first obstacle
			n = 1;
			if (d_theta_o1 < 0) {
				pw_l = 2000; //moving anti-clockwise
				pw_r = 2000;
			}
			else {
				pw_l = 1000; //moving clockwise
				pw_r = 1000;
			}
			if ((d_theta_o1 > -5) && (d_theta_o1 < 5)) {
				pw_l = 1000;//move forward
				pw_r = 2000;
			}
			if (safe_dist_o1 <= 0) {
				pw_l = 1500;//stop
				pw_r = 1500;
				init = 1;
			}
		}


		else if (dist_o1_g_bot > dist_o2_g_bot) {
			// move towards second obstacle
			n = 2;
			if (d_theta_o2 < 0) {
				pw_l = 2000; //moving anti-clockwise
				pw_r = 2000;
			}
			else {
				pw_l = 1000; //moving clockwise
				pw_r = 1000;
			}
			if ((d_theta_o2 > -5) && (d_theta_o2 < 5)) {
				pw_l = 1000;//move forward
				pw_r = 2000;
			}

			if (safe_dist_o2 <= 0) {
				pw_l = 1500;// stop
				pw_r = 1500;
				init = 1;
			}
		}
	}
	// obstacle 1 doesnt allow bot to pass so move to obs 2
	else if ((x_obs1 < 140 || x_obs1>500 || y_obs1 < 140 || y_obs1>340) &&
		(x_obs2 > 140 && x_obs2 < 500 && y_obs2 > 140 && y_obs2 < 340)) // obstacle 2 is allowing
	{
		n = 2;
		if (d_theta_o2 < 0) {
			pw_l = 2000; //moving anti-clockwise
			pw_r = 2000;
		}
		else {
			pw_l = 1000; //moving clockwise
			pw_r = 1000;
		}
		if ((d_theta_o2 > -5) && (d_theta_o2 < 5)) {
			pw_l = 1000;//move forward
			pw_r = 2000;
		}

		if (safe_dist_o2 <= 0) {
			pw_l = 1500;// stop
			pw_r = 1500;
			init = 1;
		}
	}
	// obstacle 2 doesnt allow bot to pass so move to obs 1
	else if ((x_obs2 < 140 || x_obs2>500 || y_obs2 < 140 || y_obs2>340) &&
		(x_obs1 > 140 && x_obs1 < 500 && y_obs1 > 140 && y_obs1 < 340)) // obstacle 1 is allowing
	{
		n = 1;
		if (d_theta_o1 < 0) {
			pw_l = 2000; //moving anti-clockwise
			pw_r = 2000;
		}
		else {
			pw_l = 1000; //moving clockwise
			pw_r = 1000;
		}
		if ((d_theta_o1 > -5) && (d_theta_o1 < 5)) {
			pw_l = 1000;//move forward
			pw_r = 2000;
		}
		if (safe_dist_o1 <= 0) {
			pw_l = 1500;//stop
			pw_r = 1500;
			init = 1;
		}
	}
	else {// printing out error message of obstacle inital position
		cout << "Obstacle is not well placed, Robot can't revolve without getting off screen";
		pause();
	}
}

//for optional gradient background - t values will have to change depending on what background is chosen
void imageProcessing_additional(int& tvalue, image& rgb, image& rgb0, image& a, image& b) 
{

	//threshold value
	tvalue = 70;

	// convert RGB image to greyscale image
	copy(rgb, a); //grey scale 

	scale(a, b);// scale the image to enhance contrast
	copy(b, a); // put result back into image a

	lowpass_filter(a, b); // Applying low pass filter
	copy(b, a);// put result back into image a

	threshold(a, b, tvalue); // use threshold function to make a binary image (0,255)
	copy(b, a); // put result back into image a

	invert(a, b); //invert the image
	copy(b, a);  // put result back into image a

	erode(a, b); // perform an erosion function to remove noise (small objects)
	copy(b, a);  // put result back into image a

	dialate(a, b); // perform a dialation function to fill in and grow the objects
	copy(b, a);  // put result back into image a

	dialate(a, b); // perform a dialation again
	copy(b, a);  // put result back into image a
}


//--------------------------------------END--------------------------------------------