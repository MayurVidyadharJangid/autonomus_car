

// include all necessary files and libraries for robot simulations
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "Attack_Functions.h"
#include "vision_simulation.h"
#include "timer.h"

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

//intializes robot simulation (written by sylvia)

void initialize_model(int& width1, int& height1, double& D, double& Lx, double& Ly, double& Ax, double& Ay, double& alpha_max, int& n_robot, int& tvalue)
{
	// note that the vision simulation library currently assumes an image size of 640x480
	width1 = 640;
	height1 = 480;

	// set robot model parameters //
	D = 121.0; //distance between front wheels (pixels)

	// position of laser in local robot coordinates (pixels)
	Lx = 31.0; //assume in local coord the robot is pointing in the x direction
	Ly = 0.0; //assume in local coord the robot is pointing in the x direction

	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;

	alpha_max = 3.14159 / 2; // max range of laser / gripper (rad)

	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;

	//threshold value
	tvalue = 80;
}

// sets the type image type and size (from week 11 vision part 7 ex 9)
int activate(image& a, image& b, image& rgb, image& rgb0, image& label, int IMAGE_HEIGHT, int IMAGE_WIDTH) {

	// set the type and size of the images
	a.type = GREY_IMAGE;
	a.width = IMAGE_WIDTH;
	a.height = IMAGE_HEIGHT;

	b.type = GREY_IMAGE;
	b.width = IMAGE_WIDTH;
	b.height = IMAGE_HEIGHT;

	rgb.type = RGB_IMAGE;
	rgb.width = IMAGE_WIDTH;
	rgb.height = IMAGE_HEIGHT;

	rgb0.type = RGB_IMAGE;
	rgb0.width = IMAGE_WIDTH;
	rgb0.height = IMAGE_HEIGHT;

	label.type = LABEL_IMAGE;
	label.width = IMAGE_WIDTH;
	label.height = IMAGE_HEIGHT;

	allocate_image(a);
	allocate_image(b);
	allocate_image(rgb);
	allocate_image(rgb0);
	allocate_image(label);

	return 0;

}

//performs all the necessary image processing functions (from week 11 vision part 7 ex 6)
void image_processing(int tvalue, image& rgb, image& rgb0, image& a, image& b) {
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


// This fuction find the sizes of all labeled objects)
void label_size(image& label_image, image& rgb, int label_num, int& N) {
	int height = label_image.height;
	int width = label_image.width;
	i2byte* pl = (i2byte*)label_image.pdata;  // Pointer to the label image data
	ibyte* prgb = (ibyte*)rgb.pdata;  // Pointer to the RGB image data
	N = 0;

	for (int k = 0; k < width * height; k++) { // Loop over each pixel
		int label = *pl;  // Get the label of the current pixel

		if (label == label_num) {
			N++;  // Count this pixel if it matches the label of interest
			//Process the RGB values
			ibyte B = *prgb;       // Blue component
			ibyte G = *(prgb + 1); // Green component
			ibyte R = *(prgb + 2); // Red component
			//cout << "Pixel " << k << ": Label = " << label << ", RGB = (" << int(B) << ", " << int(G) << ", " << int(R) << ")" << endl;
		}

		// Increment pointers
		pl++;
		prgb += 3;  // Move to the next pixel in the RGB image (3 bytes per pixel)
	}
}

// Functions related to centroid calculation

//Function calculates moments for red, green, blue and orange
void calculate_color_moments(ibyte* p, double& m, double& mi, double& mj, int i, int j, int color) {
	// Extract color components from the current pixel
	ibyte B = p[0], G = p[1], R = p[2];
	double colorMass = 0;  // Will store the mass (intensity) of the color if conditions are met
	
	// Check which color to process based on the index provided
	if (color == 0) { // Red
		if (B < 100 && R > 200 && G < 100) {
			colorMass = R;  // Set the mass to the red component's intensity if conditions are met
		}
	}
	else if (color == 1) { // Green
		if (B < 150 && R < 100 && G > 150) {
			colorMass = G;  // Set the mass to the green component's intensity if conditions are met
		}
	}
	else if (color == 2) { // Blue
		if (B > 210 && R < 50 && G < 170) {
			colorMass = B;  // Set the mass to the blue component's intensity if conditions are met
		}
	}
	else if (color == 3) { // Orange
		if (B < 150 && R > 180 && G > 180) {
			colorMass = (R + G) / 2;  // Set the mass to the average of red and green components if conditions are met
		}
	}

	// If a color mass was assigned (i.e., conditions met for one of the colors)
	if (colorMass > 0) {
		m += colorMass;            // Add the color mass to the total mass
		mi += i * colorMass;       // Add weighted x-coordinate to the total moment in x-direction
		mj += j * colorMass;       // Add weighted y-coordinate to the total moment in y-direction
	}
}

//Helper function related to finding centroid of obstacles
void process_large_labels(image& a, image& label, int label_num,
	double& obs1_ic, double& obs1_jc,
	double& obs2_ic, double& obs2_jc, int& initializer) {
	// Check the state of 'initializer' to determine which observation set to update
	if (initializer == 0) {
		centroid(a, label, label_num, obs1_ic, obs1_jc);  // Calculate centroid for the first observation set
		initializer = 2;  // Change the state to process the next large label differently
	}
	else if (initializer == 2) {
		centroid(a, label, label_num, obs2_ic, obs2_jc);  // Calculate centroid for the second observation set
		initializer = 0;  // Reset the initializer for future use
	}
}

//Helper function to help find centroid of circles on robots
void process_medium_labels(image& rgb, double& r_ic, double& r_jc, double& g_ic, double& g_jc, double& b_ic, double& b_jc, double& o_ic, double& o_jc, double eps) {
	double m[4] = { 0 }, mi[4] = { 0 }, mj[4] = { 0 };  // Arrays to hold total mass and moments for each color
	int width = rgb.width, height = rgb.height;  // Dimensions of the image
	ibyte* p = (ibyte*)rgb.pdata;  // Pointer to the pixel data of the image

	// Iterate over all pixels in the image
	for (int j = 0; j < height; j++) {
		for (int i = 0; i < width; i++) {
			int k = i + width * j;  // Calculate the linear index of the pixel
			ibyte* currentPixel = p + 3 * k;  // Pointer to the current pixel (3 bytes per pixel for RGB)
			// Process each color for the current pixel
			for (int color = 0; color < 4; color++) {
				calculate_color_moments(currentPixel, m[color], mi[color], mj[color], i, j, color);
			}
		}
	}
	// Update centroids for each color based on the calculated moments and masses
	r_ic = mi[0] / (m[0] + eps);
	r_jc = mj[0] / (m[0] + eps);
	g_ic = mi[1] / (m[1] + eps);
	g_jc = mj[1] / (m[1] + eps);
	b_ic = mi[2] / (m[2] + eps);
	b_jc = mj[2] / (m[2] + eps);
	o_ic = mi[3] / (m[3] + eps);
	o_jc = mj[3] / (m[3] + eps);
}

//Finds centroids for robots and obstacles using helper functions 
int centroids(
	image a, image rgb, image label, int& nlabels,
	double& r_ic, double& r_jc,
	double& g_ic, double& g_jc,
	double& o_ic, double& o_jc,
	double& b_ic, double& b_jc, int N_obs) {
	int N, initializer = 0;  // N is the count of pixels for the current label, initializer for control flow
	double eps = 1.0e-10;  // Small epsilon value to prevent division by zero
	double obs1_ic = 0, obs1_jc = 0, obs2_ic = 0, obs2_jc = 0;  // Variables to hold coordinates of observations

	// Loop through all labels
	for (int n = 1; n <= nlabels; n++) {
		label_size(label, rgb, n, N);  // Get the size of the current label

		// Process based on the size of the label
		if (N > 2000) {
			process_large_labels(a, label, n, obs1_ic, obs1_jc, obs2_ic, obs2_jc, initializer);
		}
		else if (N > 1000 && N < 1999) {
			//draw_point_rgb(rgb,r_ic,r_jc, 0, 0,255);
			//draw_point_rgb(rgb, g_ic, g_jc, 0, 0, 255);
			//draw_point_rgb(rgb, b_ic, b_jc, 0, 0, 255);
			//draw_point_rgb(rgb, o_ic, o_jc, 0, 0, 255);
			process_medium_labels(rgb, r_ic, r_jc, g_ic, g_jc, b_ic, b_jc, o_ic, o_jc, eps);
		}
	}
	return 0;  // Return success
}

//determines angle that each robot makes with the x axis and locates the robots (edited by sophia)
int position_angle(double& ic1, double& ic2, double& ic3, double& ic4, double& jc1, double& jc2, double& jc3, double& jc4,
	double& x1, double& y1, double& x2, double& y2, double& theta1, double& theta2){
	x1 = (ic1 + ic2) / 2; //calculates average x value of centroid of red dot and green dot ROBOT A
	y1 = (jc1 + jc2) / 2;//calculates average y value of centroid of red dot and green dot ROBOT A
	x2 = (ic3 + ic4) / 2; //calculates average x value of centroid of blue dot and orange dot ROBOT B
	y2 = (jc3 + jc4) / 2; //calculates average y value of centroid of blue dot and orange dot ROBOT B

	//calculate angle of ROBOT A using pythag
	double jdif_a = jc2 - jc1; //calculates height of triangle
	double idif_a = ic2 - ic1; //calculates base of triangle
	theta1 = atan2(jdif_a, idif_a); //find angle that ROBOT A makes with horizontal 
	theta1 = (theta1 * 180 / 3.1415) + (theta1 > 0 ? 0 : 360); //convert to degrees and check if valid

	// calculate angle of ROBOT B using pythag
	double jdif_b = jc4 - jc3; //calculates height of triangle
	double idif_b = ic4 - ic3; //calculates base of triangle
	theta2 = atan2(jdif_b, idif_b); //find angle that ROBOT B makes with horizontal 
	theta2 = (theta2 * 180 / 3.1415) + (theta2 > 0 ? 0 : 360); //convert to degrees and check if valid
	
			
	return 0;
}

//used to check whether obstacle is in between two robots (edited by sophia)
//angles of triangle = obstacle 1/obstacle2 + ROBOT A + robot b
int triangle_angles(double x1, double y1, double x2, double y2, double obs1_ic, double obs1_jc,
	double obs2_ic, double obs2_jc, double& d_obs1_a, double& d_obs2_a,double& d_obs_a, double& obs_ic,
	double& obs_jc, double& theta_shoot_gap_abs) {
	
	//define local variable used in function
	double theta_obs_a, theta_obs_b, theta_shoot_gap;

	//using pythag to find distance between obstacle 1/2 and ROBOT A
	d_obs1_a = sqrt(abs(pow(obs1_ic - x1, 2) + pow(obs1_jc - y1, 2))); //c = sqrt(a^2 + b^2) 
	d_obs2_a = sqrt(abs(pow(obs2_ic - x1, 2) + pow(obs2_jc - y1, 2)));

	//if obstacle 1 is closer to ROBOT A than obstacle 2 is
	if (d_obs1_a < d_obs2_a) {
		//consider obstacle 1
		obs_ic = obs1_ic;
		obs_jc = obs1_jc;
		d_obs_a = d_obs1_a;
	}
	//else obstacle 2 is closer
	else {
		//consider obstacle 2
		obs_ic = obs2_ic;
		obs_jc = obs2_jc;
		d_obs_a = d_obs2_a;
	}

	//find angle that line connecting ROBOT A and closest obstacle makes with horizontal 
	theta_obs_a = (atan2(obs_jc - y1, obs_ic - x1));
	theta_obs_a = (theta_obs_a * 180 / 3.1415) + (theta_obs_a > 0 ? 0 : 360); //convert to degrees, check validity 
	//find angle that line connecting ROBOT B and closest obstacle to ROBOT A makes with horizontal 
	theta_obs_b = (atan2(obs_jc - y2, obs_ic - x2));
	theta_obs_b = (theta_obs_b * 180 / 3.1415) + (theta_obs_b > 0 ? 0 : 360); //convert to degrees, check validity

	//calculate possible angle at which laser from ROBOT A will reach ROBOT B
	theta_shoot_gap = theta_obs_a - theta_obs_b;
	theta_shoot_gap_abs = abs(theta_shoot_gap);

	return 0;

}


//draws circle around obstacle to move ROBOT A around obstacle (adjusted by sophia)
int obstacle_avoid(double& g_ic, double& g_jc, double& obs_ic, double& obs_jc,double& d_obs_a, 
	double& theta1,double& x2,double& y2, double& p1x,double& p1y,
	double& p2x, double& p2y, int& pw_r,int& pw_l) {
	
	//define a constant for the  distance from the obstacle
	double c = 200;
	//calculate the angle between ROBOT A and the closest obstacle
	double theta_obs = atan2(obs_jc - g_jc, obs_ic - g_ic);

	//calculate the positions of two points (p1 and p2) on a circle around the obstacle
	p1x = (d_obs_a / 6) * cos(theta_obs) + c * cos(theta_obs + 3.1415 / 2) + g_ic;
	p1y = (d_obs_a / 6) * sin(theta_obs) + c * sin(theta_obs + 3.1415 / 2) + g_jc;
	p2x = (d_obs_a / 6) * cos(theta_obs) + c * cos(theta_obs + 3 * 3.1415 / 2) + g_ic;
	p2y = (d_obs_a / 6) * sin(theta_obs) + c * sin(theta_obs + 3 * 3.1415 / 2) + g_jc;

	//calculate the distances between these two points and the position of ROBOT B
	double d_p1_b = sqrt(pow(p1x - x2, 2) + pow(p1y - y2, 2));
	double d_p2_b = sqrt(pow(p2x - x2, 2) + pow(p2y - y2, 2));
	//calculate the distances between these two points and the position of ROBOT A
	double d_p1_a = sqrt(pow(p1x - g_ic, 2) + pow(p1y - g_jc, 2));
	double d_p2_a = sqrt(pow(p2x - g_ic, 2) + pow(p2y - g_jc, 2));

	//choose the closest point to ROBOT B
	double d_p_b, d_p_a;
	if (d_p1_b < d_p2_b) {
		p1x = (d_obs_a / 6) * cos(theta_obs) + c * cos((theta_obs + 3.1415 / 2)) + g_ic;
		p1y = (d_obs_a / 6) * sin(theta_obs) + c * sin((theta_obs + 3.1415 / 2)) + g_jc;
		d_p_b = d_p1_b;
		d_p_a = d_p1_a;
	}
	else {
		p2x = (d_obs_a / 6) * cos(theta_obs) + c * cos(theta_obs + 3 * 3.1415 / 2) + g_ic;
		p2y = (d_obs_a / 6) * sin(theta_obs) + c * sin(theta_obs + 3 * 3.1415 / 2) + g_jc;
		d_p_b = d_p2_b;
		d_p_a = d_p2_a;
	}

	//calculate the angle between the closest point on the obstacle's circle and ROBOT A
	double theta_p_a = atan2((d_p_b == d_p1_b ? p1y - g_jc : p2y - g_jc), (d_p_b == d_p1_b ? p1x - g_ic : p2x - g_ic)) * 180 / 3.1415;
	theta_p_a = (theta_p_a < 0) ? theta_p_a + 360 : theta_p_a;

	//calculate the angle difference between ROBOT A's direction and the desired direction to avoid the obstacle
	double theta_pxx_b_gap = theta_p_a - theta1;
	double theta_pxx_b_gap_abs = abs(theta_pxx_b_gap);

	//adjust ROBOT A's movement based on the angle difference and the distances involved
	if (theta_pxx_b_gap_abs > 7) {
		//rotate left or right
		pw_l = (theta_pxx_b_gap > 0) ? 1750 : 1000;
		pw_r = pw_l;
	}
	else if (d_p_a > 0) {
		//move forward
		pw_l = 1000;
		pw_r = 2000;
	}

	return 0;

}
// Controlling robot to fire laser when there is a clear shot
void control_robot(double& x1, double& y1, double& x2, double& y2,
	double& theta1, double& theta_gap, double& theta, 
	double& distance_a_b, int& pw_r, int& pw_l, int& laser)
{
	int tolerance = 2; // tolerance for angle difference
	
	//calculate the angle between ROBOT A and ROBOT B
	theta = atan2(y2 - y1, x2 - x1) * 180 / 3.1415;
	theta = (theta < 0) ? theta + 360 : theta;

	//calculate the distance between ROBOT A and ROBOT B
	distance_a_b = hypot(x2 - x1, y2 - y1);

	//calculate the angle difference between ROBOT A orientation and ROBOT B direction
	theta_gap = theta - theta1;
	double theta_gap_abs = abs(theta_gap);

	//default values for wheel power
	pw_l = 1500;
	pw_r = 1500;

	//control ROBOT A movement based on the angle difference and distance from ROBOT B
	if ((theta_gap < 0) && (theta_gap_abs > tolerance)) {
		//rotate right
		pw_l = 1000;
		pw_r = 1000;
	}
	else if ((theta_gap >= 0) && (theta_gap_abs > tolerance)) {
		//rotate left
		pw_l = 2000;
		pw_r = 2000;
	}
	else if (distance_a_b > 250) {
		//move forward
		pw_l = 1000;
		pw_r = 2000;
	}
	else if (distance_a_b < 150) {
		//move backward
		pw_l = 2000;
		pw_r = 1000;
	}
	else if ((theta_gap_abs < tolerance) && (distance_a_b <= 250) && (distance_a_b >= 105)) {
		//stop and fire laser when aligned with ROBOT B and at a suitable distance
		pw_l = 1500;
		pw_r = 1500;
		laser = 1; //fire laser
		
	}
	
}


// Challenge File: For optional gradient background
void image_processing_additional(int& tvalue, image & rgb, image & rgb0, image & a, image & b) {

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

