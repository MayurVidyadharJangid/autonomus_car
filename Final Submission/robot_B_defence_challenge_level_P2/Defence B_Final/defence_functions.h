const double PI = 3.14159;

int centroid_rgb_image(image& rgb, double& ic, double& jc, int value);

void image_processing(image& rgb,image& a,double xi[],double yi[], double xo[], double yo[]);

void color_check(image& ai, image& rgbi, image& rgbi2, double xo[],double yo[]);

void size_check(image& bi, image& rgbi, double xin[], double yin[],double xo[], double yo[]);

void calc_distance(double x_1, double y_1, double x_2, double y_2, double& dist_xy);

void calc_angle(double x_1, double y_1, double x_2, double y_2, double& theta);

void aligning(double ig,double jg,double ir,double jr,double io,double jo,double ib,
double jb,double i_nearest_obs,double j_nearest_obs,int& pw_l,int& pw_r,int& init);	

void stop90deg(double ig,double jg,double ir,double jr,double i_nearest_obs,
double j_nearest_obs,int& pw_l,int& pw_r,int& init);

void round_about(double ig, double jg, double ir, double jr,double io,double jo,double ib,
double jb, double i_nearest_obs, double j_nearest_obs, int& pw_l, int& pw_r, int& init);

void stop_at_zero(double ig,double jg,double ir,double jr,double i_nearest_obs,double j_nearest_obs,
double io, double jo, double ib,double jb,int& pw_l,int& pw_r,int& init,double& prev_io,double& prev_jo);

void restart(double io,double jo,int& init,double& prev_io,double& prev_jo);

void go_to_obstacle(double ig,double jg,double ir,double jr,double x_obs1,double y_obs1,
					int& pw_l,int& pw_r,int& init);

void go_to_obstacle(double ig,double jg,double ir,double jr,double x_obs1,double y_obs1,
					double x_obs2,double y_obs2,int& pw_l,int& pw_r,int& n,int& init);

void image_processing_additional(int& tvalue, image& rgb, image& rgb0, image& a, image& b);

