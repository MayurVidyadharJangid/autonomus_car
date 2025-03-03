
// team defined defined functions
void initialize_model(int& width1, int& height1, double& D, double& Lx, double& Ly, double& Ax, double& Ay, double& alpha_max, int& n_robot, int& tvalue);

int activate(image& a, image& b, image& rgb, image& rgb0, image& label, int IMAGE_HEIGHT, int IMAGE_WIDTH);

void image_processing(int tvalue, image& rgb, image& rgb0, image& a, image& b);

void label_size(image& label_image, int label_num, int& N);

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


int position_angle(double& ic1, double& ic2, double& ic3, double& ic4, double& jc1, double& jc2, double& jc3, double& jc4,
	double& x1, double& y1, double& x2, double& y2, double& theta1, double& theta2);

int triangle_angles(double x1, double y1, double x2, double y2, double obs1_ic, double obs1_jc,
	double obs2_ic, double obs2_jc, double& d_obs1_a, double& d_obs2_a, double& d_obs_a, double& obs_ic,
	double& obs_jc, double& theta_shoot_gap_abs);

int triangle_angles_d(double& x1, double& y1, double& x2, double& y2, double& obs1_ic, double& obs1_jc,
	double& obs2_ic, double& obs2_jc, double& d_obs1_b, double& d_obs2_b, double& d_obs_b,
	double& obs_ic, double& obs_jc, double& theta_shoot_gap_abs);

int obstacle_avoid(double& g_ic, double& g_jc, double& obs_ic, double& obs_jc, double& d_obs_a,
	double& theta1, double& x2, double& y2, double& p1x, double& p1y,
	double& p2x, double& p2y, int& pw_r, int& pw_l);

int obstacle_avoid_defence(double& o_ic, double& o_jc, double& g_ic, double& g_jc, double& theta2,
	double& d_obs_b, double& obs_ic, double& obs_jc, double& px, double& py, double& p1x, double& p1y,
	double& p2x, double& p2y, int& pw_r, int& pw_l);

int complete_defence(double& o_ic, double& o_jc, double& g_ic, double& g_jc, double& theta2,
	double& px, double& py, int& pw_r, int& pw_l);

void control_robot(double& x1, double& y1, double& x2, double& y2,
	double& theta1, double& theta_gap, double& theta,
	double& distance_a_b, int& pw_r, int& pw_l, int& laser);

void image_processing_additional(int& tvalue, image& rgb, image& rgb0, image& a, image& b);

