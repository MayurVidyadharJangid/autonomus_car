// Player 2 - Attack Robot B 
// include all necessary files and libraries for robot simulations
#include <cstdio>
#include <iostream>
#include <fstream>
#include <Windows.h>
#include "image_transfer.h"
#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "update_simulation.h"
#include "timer.h"
#include "Attack_Functions.h"

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

extern robot_system S1; //needed for simulation

int main()
{
    //all of these initialized variables carried over from lecture code (see week 12 manual opponent codes)
    double x0, y0, theta0, max_speed, opponent_max_speed;
    int pw_l, pw_r, pw_laser, laser;
    int width1, height1;
    int n_robot, tvalue;
    double x_obs[50], y_obs[50];
    double D, Lx, Ly, Ax, Ay, alpha_max;
    double tc, tc0; // clock time
    int mode;
   
    //Initializing coloured robot centroids
    double r_ic, r_jc;
    double g_ic, g_jc;
    double o_ic, o_jc;
    double b_ic, b_jc;
   
    //Initializing simulation
    initialize_model(width1, height1, D, Lx, Ly, Ax, Ay, alpha_max, n_robot, tvalue);
   
    // number of obstacles
    const int N_obs = 2;

    // Obstacles coordinates 
    double obs1_ic, obs1_jc, obs2_ic, obs2_jc;

    // obstacle locations
    // -- you must set one location for each obstacle
    x_obs[0] = 250; // pixels
    y_obs[0] = 320; // pixels

    x_obs[1] = 180; // pixels
    y_obs[1] = 180; // pixels


    cout << "\npress space key to begin program.";
    pause();

    // activating the vision simulation library
    activate_vision();

    char obstacle_file[N_obs][S_MAX] = {
         "obstacle_black.bmp",     
         "obstacle_black.bmp"
    };

    activate_simulation(width1, height1, x_obs, y_obs, N_obs,
        "robot_A.bmp", "robot_B.bmp", "background.bmp", obstacle_file
        , D, Lx, Ly, Ax, Ay, alpha_max, n_robot);

    // set simulation mode 
    // mode = 0 - single player mode (manual opponent)
    // mode = 1 - two player mode, player #1 
    // mode = 2 - two player mode, player #2    
    mode = 2;
    set_simulation_mode(mode);

    // set robot initial position (pixels) and angle (rad)
    x0 = 470;
    y0 = 170;
    theta0 = 0;
    set_robot_position(x0, y0, theta0);

    // set initial inputs, straight line speed
    pw_l = 1500; // pulse width for left wheel servo (us) 
    pw_r = 1500; // pulse width for right wheel servo (us)
    pw_laser = 1500; // pulse width for laser servo (us)
    laser = 0; // laser input (0 - off, 1 - fire)
    // paramaters
    max_speed = 100; // max wheel speed of robot (pixels/s)
    opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)

    // set initial inputs
    set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);

    // NOTE: two player mode so opponent inputs are not set

    // regular vision program ////////////////////////////////

    image rgb, rgb0, a, b, label;
    int nlabels; 

    // measure initial clock time
    tc0 = high_resolution_time();

    //initializes the program, defined in my_functions.cpp (from week 11)
    activate(a, b, rgb, rgb0, label, height1, width1);

    //simulates the robots and aquires the image from simulation
    acquire_image_sim(rgb);

    image_processing(tvalue, rgb, rgb0, a, b); //image processing code 

    //image_processing_additional(tvalue, rgb, rgb0, a, b); //image processsing written by Sophia for challenge level
     
    label_image(a, label, nlabels); //labels objects in a binary image, labels go from 1 to nlabels (defined in vision.cpp)
    

    // Finding centroids of labelled images
    
    void calculate_color_moments(ibyte* p, double& m, double& mi, double& mj, int i, int j, int color);

    void process_large_labels(image& a, image& label, int label_num,
        double& obs1_ic, double& obs1_jc,
        double& obs2_ic, double& obs2_jc, int& initializer);

    void process_medium_labels(image& rgb, double& r_ic, double& r_jc, double& g_ic, double& g_jc, double& b_ic, double& b_jc, double& o_ic, double& o_jc, double eps);

    int centroids(image a, image rgb, image label, int& nlabels,
        double& r_ic, double& r_jc,
        double& g_ic, double& g_jc,
        double& o_ic, double& o_jc,
        double& b_ic, double& b_jc, int N_obs);

    //draw_point_rgb(rgb, obs1_ic, obs1_jc, 255, 0, 0);

    join_player();

    // measure initial clock time
    tc0 = high_resolution_time();
 
    
    while (1) {
        // Define local variables  
        double x1, y1, x2, y2, theta1, theta2; // Used to find robot position
 //       double obs1_ic, obs1_jc, obs2_ic, obs2_jc;
        double d_obs1_a, d_obs2_a, d_obs_a, theta_shoot_gap_abs, obs_ic, obs_jc; // Used to orient obstacles
        double p1x, p1y, p2x, p2y, theta_gap, theta, distance_a_b; // Used to control robot
        double ic[100], jc[100]; // Array to store centroid coordinates for each label

        // Simulates the robots and acquires the image from simulation
        acquire_image_sim(rgb);
        tc = high_resolution_time() - tc0;

        // Vision code
        image_processing(tvalue, rgb, rgb0, a, b);
        label_image(a, label, nlabels);

        // Calculate centroids for each label and visualize them
        for (int i = 1; i <= nlabels; ++i) {
            double temp_ic, temp_jc; // Temporary variables for centroid coordinates
            if (centroid(a, label, i, temp_ic, temp_jc) == 0) {
                ic[i] = temp_ic;
                jc[i] = temp_jc;
                //cout << "Centroid for label " << i << ": (" << ic[i] << ", " << jc[i] << ")\n";
                //draw_point_rgb(rgb, static_cast<int>(ic[i]), static_cast<int>(jc[i]), 255, 0, 0);
            }
        }

        void calculate_color_moments(ibyte * p, double& m, double& mi, double& mj, int i, int j, int color);

        void process_large_labels(image & a, image & label, int label_num, double& obs1_ic, double& obs1_jc,
            double& obs2_ic, double& obs2_jc, int& initializer);

        void process_medium_labels(image & rgb, double& r_ic, double& r_jc, double& g_ic, double& g_jc, double& b_ic, double& b_jc, double& o_ic, double& o_jc, double eps);

        // Centroids function to calculate positions of robots and obstacles
        centroids(a, rgb, label, nlabels, r_ic, r_jc, g_ic, g_jc, o_ic, o_jc, b_ic, b_jc, N_obs);

        // Get location and angle (wrt horizontal) of each robot
        position_angle(r_ic, g_ic, b_ic, o_ic, r_jc, g_jc, b_jc, o_jc, x1, y1, x2, y2, theta1, theta2);

        // Determine which obstacle is closest and if there is a clear shot to ROBOT B
        triangle_angles(x1, y1, x2, y2, obs1_ic, obs1_jc, obs2_ic, obs2_jc,
            d_obs1_a, d_obs2_a, d_obs_a, obs_ic, obs_jc, theta_shoot_gap_abs);

        // Decision-making based on obstacle proximity and angle gap
        if ((d_obs_a < 200) && (theta_shoot_gap_abs > 100) && (theta_shoot_gap_abs < 250)) {
            //cout << "Obstacle avoidance triggered." << endl;
            // Call obstacle avoidance strategy
            obstacle_avoid(g_ic, g_jc, obs_ic, obs_jc, d_obs_a, theta1, o_ic, o_jc,
                p1x, p1y, p2x, p2y, pw_r, pw_l);
        }
        else {
           // cout << "Moving towards opponent." << endl;
            // Move towards opponent and attempt to shoot
            control_robot(g_ic, g_jc, x2, y2, theta1, theta_gap, theta,
                distance_a_b, pw_r, pw_l, laser);
        }

        // Update robot inputs
        set_inputs(pw_l, pw_r, pw_laser, laser, max_speed);

        // Display the robot vision processing results
		// NOTE: only one player can use image_view()
//		view_rgb_image(rgb);
        // Simulate at 100 frames per second
        Sleep(10);
    }

    // free the image memory before the program completes
    free_image(a);
    free_image(b);
    free_image(label);
    free_image(rgb);
    free_image(rgb0);

    //End of sequence

    deactivate_vision();
    deactivate_simulation();

    cout << "\ndone.\n";
    return 0;
}
