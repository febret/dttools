#define _CRT_SECURE_NO_WARNINGS

/* standard global include files */
#include <stdio.h>
#include <math.h>
#include <string.h>
#ifndef WIN32
#include <cmath>
using std::isnan;
#define _isnan isnan
#define _finite finite
#endif

/* mbio include files */
#include "mb_fbt.h"

#include "dttypes.h"

/* raytracing defines */
#define	MB_RT_GRADIENT_TOLERANCE    0.00001
#define	MB_RT_LAYER_HOMOGENEOUS	    0
#define	MB_RT_LAYER_GRADIENT	    1
#define	MB_RT_ERROR 0
#define	MB_RT_DOWN 1
#define	MB_RT_UP 2
#define	MB_RT_DOWN_TURN 3
#define	MB_RT_UP_TURN 4
#define	MB_RT_OUT_BOTTOM 5
#define	MB_RT_OUT_TOP 6
#define	MB_RT_NUMBER_SEGMENTS 5
#define	MB_SSV_NO_USE	    0


/* velocity model structure */
struct	velocity_model
	{
	int	number_node;
	double	*depth;
	double	*velocity;
	int	number_layer;
	int	*layer_mode;
	double	*layer_gradient;
	double	*layer_depth_center;
	double	*layer_depth_top;
	double	*layer_depth_bottom;
	double	*layer_vel_top;
	double	*layer_vel_bottom;
	};
	
/* global raytrace values */
static struct velocity_model *model;
static int	ray_status;
static int	done;
static int	outofbounds;
static int	layer;
static int	turned;
static int	number_plot_max;
static int	number_plot;
static int	sign_x;
static double	xx;
static double	zz;
static double	xf;
static double	zf;
static double	tt;
static double	dt;
static double	tt_left;
static double	vv_source;
static double	pp;
static double	xc;
static double	zc;
static double	radius;
static double	*xx_plot;
static double	*zz_plot;

/*--------------------------------------------------------------------------*/
void mb_load_svp(const char* filename, float** hDepth, float** hVelocity, int* svpSize)
{
	int error = 0;
	int verbose = 0;
	int nsvp = 0;
	char buffer[256];
	char* result;
	size_t size;
	FILE* tfp;
	float* depth;
	float* velocity;
	
	/* count the data points in the svp file */
	if ((tfp = fopen(filename, "r")) == NULL) 
	{
		fprintf(stderr,"\nUnable to Open Velocity Profile File <%s> for reading\n", filename);
		exit(error);
	}
	while ((result = fgets(buffer,256,tfp)) == buffer)
	{
		if (buffer[0] != '#') nsvp++;
	}
	fclose(tfp);
	
	/* allocate arrays for svp */
	if (nsvp > 1)
	{
		size = (nsvp+2) * sizeof(float);
		mb_malloc(verbose,size,(void **)hDepth,&error);
		mb_malloc(verbose,size,(void **)hVelocity,&error);
		
		depth = *hDepth;
		velocity = *hVelocity;
	}
	else
	{
		fprintf(stderr,"\nUnable to read data from SVP file <%s>\n",filename);
		exit(error);
	}		    
	
	/* read the data points in the svp file */
	nsvp = 0;
	if ((tfp = fopen(filename, "r")) == NULL) 
	{
		fprintf(stderr,"\nUnable to Open Velocity Profile File <%s> for reading\n",filename);
		exit(error);
	}
	while ((result = fgets(buffer,256,tfp)) == buffer)
	{
		if (buffer[0] != '#')
		{
			/* read the depth & sound speed pair */
			int mm = sscanf(buffer,"%f %f",&depth[nsvp],&velocity[nsvp]);
		
			/* update counter */
			if (mm == 2) nsvp++;
			
			/* check for nonzero initial depth & fix it if found */
			if (mm == 2 && nsvp == 1 && depth[0] != 0.0)
			{
				depth[1] = depth[0];
				velocity[1] = velocity[0];
				depth[0] = 0.0;
				nsvp++;
			}
		}
	}
	fclose(tfp);

	/* if velocity profile doesn't extend to 12000 m depth extend it to that depth */
	if (depth[nsvp-1] < 12000.0)
	{
		depth[nsvp] = 12000.0;
		velocity[nsvp] = velocity[nsvp-1];
		nsvp++;
	}
	
	*svpSize = nsvp;
}

/*--------------------------------------------------------------------------*/
int mb_rt_init(int verbose, int number_node, 
		float *depth, float *velocity, 
		void **modelptr, int *error)
{
	int	status = MB_SUCCESS;
	int	i;

	/* allocate memory for model structure */
	status = mb_malloc(verbose,sizeof(struct velocity_model),(void **)modelptr,error);

	/* set variables and allocate memory for velocity model */
	model = (struct velocity_model *) *modelptr;
	model->number_node = number_node;
	status = mb_malloc(verbose,number_node*sizeof(double),(void **)&(model->depth),error);
	status = mb_malloc(verbose,number_node*sizeof(double), (void **)&(model->velocity),error);
	model->number_layer = number_node - 1;
	status = mb_malloc(verbose,model->number_layer*sizeof(int), (void **)&(model->layer_mode),error);
	status = mb_malloc(verbose,model->number_layer*sizeof(double), (void **)&(model->layer_gradient),error);
	status = mb_malloc(verbose,model->number_layer*sizeof(double), (void **)&(model->layer_depth_center),error);
	if (status == MB_SUCCESS)
	{
		model->layer_depth_top = &model->depth[0];
		model->layer_depth_bottom = &model->depth[1];
		model->layer_vel_top = &model->velocity[0];
		model->layer_vel_bottom = &model->velocity[1];
	}

	/* put model into structure */
	for (i=0;i<number_node;i++)
	{
		model->depth[i] = depth[i];
		model->velocity[i] = velocity[i];
	}
	for (i=0;i<model->number_layer;i++)
	{
		model->layer_gradient[i] = 
			(model->layer_vel_bottom[i] - model->layer_vel_top[i])/
			(model->layer_depth_bottom[i] - model->layer_depth_top[i]);
		if (fabs(model->layer_gradient[i]) > MB_RT_GRADIENT_TOLERANCE)
		{
			model->layer_mode[i] = MB_RT_LAYER_GRADIENT;
			model->layer_depth_center[i] = 
				model->layer_depth_top[i] - 
				model->layer_vel_top[i]/
				model->layer_gradient[i];
		}
		else
		{
			model->layer_mode[i] = MB_RT_LAYER_HOMOGENEOUS;
			model->layer_depth_center[i] = 0.0;
		}
	}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_deall(int verbose, void **modelptr, int *error)
{
	char	*function_name = "mb_rt";
	int	status = MB_SUCCESS;

	/* deallocate memory for velocity model */
	model = (struct velocity_model *) *modelptr;
	status = mb_free(verbose,(void **)&(model->depth),error);
	status = mb_free(verbose,(void **)&(model->velocity),error);
	status = mb_free(verbose,(void **)&(model->layer_mode),error);
	status = mb_free(verbose,(void **)&(model->layer_gradient),error);
	status = mb_free(verbose,(void **)&(model->layer_depth_center),error);
	status = mb_free(verbose,(void **)modelptr,error);

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt(int verbose, void *modelptr, 
	double source_depth, double source_angle, double end_time, 
	int ssv_mode, double surface_vel, double null_angle, 
	int nplot_max, int *nplot, double *xplot, double *zplot, 
	double *x, double *z, double *travel_time, int *ray_stat, int *error)
{
	int	status = MB_SUCCESS;
	double	diff_angle;
	double	vel_ratio;
	int	i;

	/* get pointer to velocity model */
	model = (struct velocity_model *) modelptr;

	/* prepare the ray */
	layer = -1;
	for (i=0;i<model->number_layer;i++)
		{
		if (source_depth >= model->layer_depth_top[i] 
			&& source_depth <= model->layer_depth_bottom[i])
			layer = i;
		}
	if (layer == -1)
		{
		status = MB_FAILURE;
		*error = MB_ERROR_BAD_PARAMETER;
		return(status);
		}
	vv_source = model->layer_vel_top[layer] 
		+ model->layer_gradient[layer]*(source_depth - model->layer_depth_top[layer]);

	if (ssv_mode == MB_SSV_CORRECT && surface_vel > 0.0)
		{
		pp = sin(DTR*source_angle)/surface_vel;
		vel_ratio = MIN(1.0, pp * vv_source);
		source_angle = asin(vel_ratio) * RTD;
		}
	else if (ssv_mode == MB_SSV_INCORRECT && surface_vel > 0.0)
		{
		diff_angle = source_angle - null_angle;
		pp = sin(DTR*diff_angle)/surface_vel;
		vel_ratio = MIN(1.0, pp * vv_source);
		diff_angle = asin(vel_ratio) * RTD;
		source_angle = null_angle + diff_angle;
		}

	/* now initialize ray */
	if (source_angle > 0.0)
		sign_x = 1;
	else
		sign_x = -1;
	source_angle = fabs(source_angle);
	pp = sin(DTR*source_angle)/vv_source;
	if (source_angle < 90.0)
		{
		turned = MB_NO;
		ray_status = MB_RT_DOWN;
		}
	else if(source_angle > 90)
		{
		turned = MB_YES;
		ray_status = MB_RT_UP;
		}
	else 
	{
		turned = MB_NO;
		ray_status = MB_RT_DOWN;
		source_angle = 89;
	}
	xx = 0.0;
	zz = source_depth;
	tt = 0.0;
	tt_left = end_time;
	outofbounds = MB_NO;
	done = MB_NO;

	/* set up raypath plotting */
	number_plot_max = nplot_max;
	number_plot = 0;
	if (number_plot_max > 0)
		{
		xx_plot = xplot;
		zz_plot = zplot;
		xx_plot[0] = xx;
		zz_plot[0] = zz;
		number_plot++;
		}

	/* trace the ray */
	while (!done && !outofbounds)
		{
		/* trace ray through current layer */
		if (model->layer_mode[layer] == MB_RT_LAYER_GRADIENT 
			&& pp > 0.0)
			status = mb_rt_circular(verbose, error);
		else if (model->layer_mode[layer] == MB_RT_LAYER_GRADIENT)
			status = mb_rt_vertical(verbose, error);
		else
			status = mb_rt_line(verbose, error);

		if(status == MB_FAILURE) return status;

		/* update ray */
		tt = tt + dt;
		if (layer < 0)
			{
			outofbounds = MB_YES;
			ray_status = MB_RT_OUT_TOP;
			}
		if (layer >= model->number_layer)
			{
			outofbounds = MB_YES;
			ray_status = MB_RT_OUT_BOTTOM;
			}
		if (tt_left <= 0.0)
			done = MB_YES;

		/* reset position */
		xx = xf;
		zz = zf;
		}

	/* report results */
	*x = xx;
	*z = zz;
	*travel_time = tt;
	*ray_stat = ray_status;
	if (number_plot_max > 0)
		*nplot = number_plot;

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_circular(int verbose, int *error)
{
	int	status = MB_SUCCESS;

	/* decide which case to use */
	if (turned == MB_NO && model->layer_gradient[layer] > 0.0)
		status = mb_rt_quad1(verbose, error);
	else if (turned == MB_NO)
		status = mb_rt_quad3(verbose, error);	
	else if (turned == MB_YES && model->layer_gradient[layer] > 0.0)
		status = mb_rt_quad2(verbose, error);	
	else if (turned == MB_YES)
		status = mb_rt_quad4(verbose, error);

	/* put points in plotting arrays */
	if (number_plot_max > 0)
		status = mb_rt_plot_circular(verbose, error);

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_quad1(int verbose, int *error)
{
	int	status = MB_SUCCESS;
	double	vi;
	double	ip;
	double	ipvi;
	double	beta;
	double	ivf;

	/* find circular path */
	radius = fabs(1.0 / (pp * model->layer_gradient[layer]));
	zc = model->layer_depth_center[layer];
	xc = xx + sqrt(radius * radius - (zz - zc) * (zz - zc));
	vi = model->layer_vel_top[layer] 
		+ (zz - model->layer_depth_top[layer]) 
		* model->layer_gradient[layer];
	ip = 1.0 / pp;
	ipvi = ip/vi;
	beta = log(ipvi + sqrt(ipvi * ipvi - 1.0));

	if(_isnan(beta))
	{
		return MB_FAILURE;
	}

	/* Check if ray turns in layer */
	if (zc + radius < model->layer_depth_bottom[layer])
		{
		/* ray can turn in this layer */
		dt = fabs(beta / model->layer_gradient[layer]);

		/* raypath ends before turning */
		if (dt >= tt_left)
			{
			mb_rt_get_depth(verbose, beta, -1, 1, &zf, error);
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			dt = tt_left;
			tt_left = 0.0;
			}

		/* raypath turns */
		else
			{
			ivf = 1.0 / model->layer_vel_top[layer];
			dt = fabs((log(ip * ivf + 
				ip * sqrt(ivf * ivf - pp * pp)) + beta)
				/ model->layer_gradient[layer]);

			/* ray turns and exits layer before 
				exhausting tt_left */
			if (dt <= tt_left)
				{
				turned = MB_YES;
				ray_status = MB_RT_UP_TURN;
				zf = model->layer_depth_top[layer];
				xf = xc + sqrt(radius * radius 
					- (zf - zc) * (zf - zc));
				tt_left = tt_left - dt;
				layer--;
				}
			/* ray turns and exhausts tt_left 
				before exiting layer */
			else if (dt > tt_left)
				{
				turned = MB_YES;
				ray_status = MB_RT_UP_TURN;
				mb_rt_get_depth(verbose, beta, 1, -1, &zf, error);
				xf = xc + sqrt(radius * radius 
					- (zf - zc) * (zf - zc));
				dt = tt_left;
				tt_left = 0.0;
				}
			}
		}
	else
		{
		/* ray cannot turn in this layer */
		ivf = 1.0 / model->layer_vel_bottom[layer];
		dt = fabs((log(ip * ivf + 
			ip * sqrt(ivf * ivf - pp * pp)) - beta)
			/ model->layer_gradient[layer]);
		/* ray exits layer before exhausting tt_left */
		if (dt <= tt_left)
			{
			zf = model->layer_depth_bottom[layer];
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			tt_left = tt_left - dt;
			layer++;
			}
		/* ray exhausts tt_left before exiting layer */
		else if (dt > tt_left)
			{
			turned = MB_YES;
			ray_status = MB_RT_UP_TURN;
			mb_rt_get_depth(verbose, beta, -1, 1, &zf, error);
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			dt = tt_left;
			tt_left = 0.0;
			}
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_quad2(int verbose, int *error)
{
	char	*function_name = "mb_rt_quad2";
	int	status = MB_SUCCESS;
	double	vi;
	double	ip;
	double	ipvi;
	double	beta;
	double	ivf;

	/* find circular path */
	radius = fabs(1.0 / (pp * model->layer_gradient[layer]));
	zc = model->layer_depth_center[layer];
	xc = xx - sqrt(radius * radius - (zz - zc) * (zz - zc));
	vi = model->layer_vel_top[layer] 
		+ (zz - model->layer_depth_top[layer]) 
		* model->layer_gradient[layer];
	ip = 1.0 / pp;
	ipvi = ip/vi;
	beta = log(ipvi + sqrt(ipvi * ipvi - 1.0));

	if(_isnan(beta))
	{
		return MB_FAILURE;
	}

	/* Check if ray ends in layer */
	ivf = 1.0 / model->layer_vel_top[layer];
	dt = fabs((log(ip * ivf + 
		ip * sqrt(ivf * ivf - pp * pp)) - beta)
		/ model->layer_gradient[layer]);
	/* ray exits layer before exhausting tt_left */
	if (dt <= tt_left)
		{
		zf = model->layer_depth_top[layer];
		xf = xc + sqrt(radius * radius 
			- (zf - zc) * (zf - zc));
		tt_left = tt_left - dt;
		layer--;
		}
	/* ray exhausts tt_left before exiting layer */
	else if (dt > tt_left)
		{
		mb_rt_get_depth(verbose, beta, 1, 1, &zf, error);
		xf = xc + sqrt(radius * radius 
			- (zf - zc) * (zf - zc));
		dt = tt_left;
		tt_left = 0.0;
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_quad3(int verbose, int *error)
{
	char	*function_name = "mb_rt_quad3";
	int	status = MB_SUCCESS;
	double	vi;
	double	ip;
	double	ipvi;
	double	beta;
	double	ivf;

	/* find circular path */
	radius = fabs(1.0 / (pp * model->layer_gradient[layer]));
	zc = model->layer_depth_center[layer];
	xc = xx - sqrt(radius * radius - (zz - zc) * (zz - zc));
	vi = model->layer_vel_top[layer] 
		+ (zz - model->layer_depth_top[layer]) 
		* model->layer_gradient[layer];
	ip = 1.0 / pp;
	ipvi = ip/vi;
	beta = log(ipvi + sqrt(ipvi * ipvi - 1.0));

	if(_isnan(beta))
	{
		return MB_FAILURE;
	}

	/* Check if ray ends in layer */
	ivf = 1.0 / model->layer_vel_bottom[layer];
	dt = fabs((log(ip * ivf + 
		ip * sqrt(ivf * ivf - pp * pp)) - beta)
		/ model->layer_gradient[layer]);

	/* ray exits layer before exhausting tt_left */
	if (dt <= tt_left)
		{
		zf = model->layer_depth_bottom[layer];
		xf = xc + sqrt(radius * radius 
			- (zf - zc) * (zf - zc));
		tt_left = tt_left - dt;
		layer++;
		}
	/* ray exhausts tt_left before exiting layer */
	else if (dt > tt_left)
		{
		mb_rt_get_depth(verbose, beta, 1, 1, &zf, error);
		xf = xc + sqrt(radius * radius 
			- (zf - zc) * (zf - zc));
		dt = tt_left;
		tt_left = 0.0;
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_quad4(int verbose, int *error)
{
	char	*function_name = "mb_rt_quad4";
	int	status = MB_SUCCESS;
	double	vi;
	double	ip;
	double	ipvi;
	double	beta;
	double	ivf;

	/* find circular path */
	radius = fabs(1.0 / (pp * model->layer_gradient[layer]));
	zc = model->layer_depth_center[layer];
	xc = xx + sqrt(radius * radius - (zz - zc) * (zz - zc));
	vi = model->layer_vel_top[layer] 
		+ (zz - model->layer_depth_top[layer]) 
		* model->layer_gradient[layer];
	ip = 1.0 / pp;
	ipvi = ip/vi;
	beta = log(ipvi + sqrt(ipvi * ipvi - 1.0));

	if(_isnan(beta))
	{
		return MB_FAILURE;
	}

	/* Check if ray turns in layer */
	if (zc - radius > model->layer_depth_top[layer])
		{
		/* ray can turn in this layer */
		dt = fabs(beta / model->layer_gradient[layer]);

		/* raypath ends before turning */
		if (dt >= tt_left)
			{
			mb_rt_get_depth(verbose, beta, -1, 1, &zf, error);
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			dt = tt_left;
			tt_left = 0.0;
			}

		/* raypath turns */
		else
			{
			ivf = 1.0 / model->layer_vel_bottom[layer];
			dt = fabs((log(ip * ivf + 
				ip * sqrt(ivf * ivf - pp * pp)) + beta)
				/ model->layer_gradient[layer]);

			/* ray turns and exits layer before 
				exhausting tt_left */
			if (dt <= tt_left)
				{
				turned = MB_NO;
				ray_status = MB_RT_DOWN_TURN;
				zf = model->layer_depth_bottom[layer];
				xf = xc + sqrt(radius * radius 
					- (zf - zc) * (zf - zc));
				tt_left = tt_left - dt;
				layer++;
				}
			/* ray turns and exhausts tt_left 
				before exiting layer */
			else if (dt > tt_left)
				{
				turned = MB_NO;
				ray_status = MB_RT_DOWN_TURN;
				mb_rt_get_depth(verbose, beta, 1, -1, &zf, error);
				xf = xc + sqrt(radius * radius 
					- (zf - zc) * (zf - zc));
				dt = tt_left;
				tt_left = 0.0;
				}
			}
		}
	else
		{
		/* ray cannot turn in this layer */
		ivf = 1.0 / model->layer_vel_top[layer];
		dt = fabs((log(ip * ivf + 
			ip * sqrt(ivf * ivf - pp * pp)) - beta)
			/ model->layer_gradient[layer]);

		/* ray exits layer before exhausting tt_left */
		if (dt <= tt_left)
			{
			zf = model->layer_depth_top[layer];
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			tt_left = tt_left - dt;
			layer--;
			}
		/* ray exhausts tt_left before exiting layer */
		else if (dt > tt_left)
			{
			turned = MB_YES;
			ray_status = MB_RT_UP_TURN;
			mb_rt_get_depth(verbose, beta, -1, 1, &zf, error);
			xf = xc - sqrt(radius * radius 
				- (zf - zc) * (zf - zc));
			dt = tt_left;
			tt_left = 0.0;
			}
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_get_depth(int verbose, double beta, int dir_sign, int turn_sign, 
		double *depth, int *error)
{
	char	*function_name = "mb_rt_get_depth";
	int	status = MB_SUCCESS;
	double	alpha;
	double	velf;

	/* find depth */
	alpha = pp * exp(dir_sign * tt_left
		* fabs(model->layer_gradient[layer]) 
		+ turn_sign*beta);
	velf = 2 * alpha / (alpha * alpha + pp * pp);
	*depth = model->layer_depth_top[layer] 
		+ (velf - model->layer_vel_top[layer]) 
		/ model->layer_gradient[layer];

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_plot_circular(int verbose, int *error)
{
	char	*function_name = "mb_rt_plot_circular";
	int	status = MB_SUCCESS;
	double	ai;
	double	af;
	double	dang;
	double	angle;
	int	i;

	/* get angle range */
	ai = atan2((xx - xc), (zz - zc));
	af = atan2((xf - xc), (zf - zc));
	dang = (af - ai)/MB_RT_NUMBER_SEGMENTS;

	/* add points to plotting arrays */
	for (i=0;i<MB_RT_NUMBER_SEGMENTS;i++)
		{
		angle = ai + (i + 1) * dang;
		if (number_plot < number_plot_max)
			{
			xx_plot[number_plot] = sign_x * (xc + radius * sin(angle));
			zz_plot[number_plot] = zc + radius * cos(angle);
			number_plot++;
			}
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_line(int verbose, int *error)
{
	char	*function_name = "mb_rt_line";
	int	status = MB_SUCCESS;
	double	theta;
	double	xvel;
	double	zvel;

	/* find linear path */
	theta = asin(pp * model->layer_vel_top[layer]);
	if (turned == MB_NO)
		{
		zf = model->layer_depth_bottom[layer];
		}
	else
		{
		theta = theta + M_PI;
		zf = model->layer_depth_top[layer];
		}
	xvel = model->layer_vel_top[layer] * sin(theta);
	zvel = model->layer_vel_top[layer] * cos(theta);
	if (zvel != 0.0)
		dt = (zf - zz) / zvel;
	else
		dt = 100 * tt_left;

	/* ray exhausts tt_left before exiting layer */
	if (dt >= tt_left)
		{
		xf = xx + xvel * tt_left;
		zf = zz + zvel * tt_left;
		dt = tt_left;
		tt_left = 0.0;
		}

	/* ray exits layer before exhausting tt_left */
	else
		{
		xf = xx + xvel*dt;
		zf = zz + zvel*dt;
		tt_left = tt_left - dt;
		if (turned == MB_YES)
			layer--;
		else
			layer++;
		}

	/* put points in plotting arrays */
	if (number_plot_max > 0 && number_plot < number_plot_max)
		{
		xx_plot[number_plot] = sign_x * xf;
		zz_plot[number_plot] = zf;
		number_plot++;
		}

	return(status);
}
/*--------------------------------------------------------------------------*/
int mb_rt_vertical(int verbose, int *error)
{
	char	*function_name = "mb_rt_vertical";
	int	status = MB_SUCCESS;
	double	vi;
	double	vf;
	double	vfvi;

	/* find linear path */
	vi = model->layer_vel_top[layer] 
		+ (zz - model->layer_depth_top[layer]) 
		* model->layer_gradient[layer];
	if (turned == MB_NO)
		{
		zf = model->layer_depth_bottom[layer];
		vf = model->layer_vel_bottom[layer];
		}
	else
		{
		zf = model->layer_depth_top[layer];
		vf = model->layer_vel_top[layer];
		}
	dt = fabs(log(vf / vi) / model->layer_gradient[layer]);

	/* ray exhausts tt_left before exiting layer */
	if (dt >= tt_left)
		{
		xf = xx;
		vfvi = exp(tt_left * model->layer_gradient[layer]);
		if (turned == MB_NO)
			vf = vi * vfvi;
		else if (turned == MB_YES)
			vf = vi / vfvi;
		zf = (vf - model->layer_vel_top[layer])
			/ model->layer_gradient[layer]
			+ model->layer_depth_top[layer];
		dt = tt_left;
		tt_left = 0.0;
		}

	/* ray exits layer before exhausting tt_left */
	else
		{
		xf = xx;
		tt_left = tt_left - dt;
		if (turned == MB_YES)
			layer--;
		else
			layer++;
		}

	/* put points in plotting arrays */
	if (number_plot_max > 0 && number_plot < number_plot_max)
		{
		xx_plot[number_plot] = sign_x * xf;
		zz_plot[number_plot] = zf;
		number_plot++;
		}

	return(status);
}


void SoundVelocityProfile::load(const char* filename)
{
	int verbose = 5;
	int error = 0;
	mb_load_svp(filename, &depth, &velocity, &size);
	mb_rt_init(verbose, size, depth, velocity, &model, &error);
}
/*--------------------------------------------------------------------------*/
