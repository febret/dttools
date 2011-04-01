/* standard include files */
#include <stdio.h>
#include <math.h>

/* mbio include files */
#include "mb_fbt.h"

/*--------------------------------------------------------------------*/
int mb_takeoff_to_rollpitch(int verbose,
		double theta, double phi,
		double *pitch, double *roll,
		int *error)
{
	int	status = MB_SUCCESS;
	double	x, y, z;

	/* convert to cartesian coordinates */
	x = sin(DTR * theta) * cos(DTR * phi);
	y = sin(DTR * theta) * sin(DTR * phi);
	z = cos(DTR * theta);

	/* convert to roll-pitch coordinates */
	*roll = acos(x);
	*pitch = asin(y / sin(*roll));
	*pitch *= RTD;
	*roll *= RTD;

	/* assume success */
	*error = MB_ERROR_NO_ERROR;
	status = MB_SUCCESS;

	/* return status */
	return(status);
}
/*--------------------------------------------------------------------*/
int mb_rollpitch_to_takeoff(int verbose,
		double pitch, double roll,
		double *theta, double *phi,
		int *error)
{
	int	status = MB_SUCCESS;
	double	x, y, z;
	double	sintheta;

	/* convert to cartesian coordinates */
	x = cos(DTR * roll);
	y = sin(DTR * pitch) * sin(DTR * roll);
	z = cos(DTR * pitch) * sin(DTR * roll);

	/* convert to takeoff angle coordinates */
	*theta = acos(z);
	sintheta = sin(*theta);
	if (fabs(sintheta) < 0.00001)
		{
		*phi = 0.0;
		}
	else
		{
		*phi = atan2(y,x);
		}
	*theta *= RTD;
	*phi *= RTD;

	/* assume success */
	*error = MB_ERROR_NO_ERROR;
	status = MB_SUCCESS;

	/* return status */
	return(status);
}
/*--------------------------------------------------------------------*/
int mb_xyz_to_takeoff(int verbose,
		float x, float y, float z,
		float *theta, float *phi,
		int *error)
{
	int	status = MB_SUCCESS;
	float	aa, xx, yy, zz, rr;
	
	/* normalize cartesian coordinates */
	rr = sqrt(x * x + y * y + z * z);
	xx = x / rr;
	yy = y / rr;
	zz = z / rr;

	/* convert to takeoff angle coordinates */
	*theta = acos(zz);
	if (zz < 1.0)
	    aa = yy / sin(*theta);
	else 
	    aa = 0.0;
	if (aa > 1.0)
	    *phi = 0.5 * M_PI;
	else if (aa < -1.0)
	    *phi = -0.5 * M_PI;
	else
	    *phi = asin(aa);
	*theta *= RTD;
	*phi *= RTD;
	if (xx < 0.0)
		*phi = 180.0f - *phi;

	/* assume success */
	*error = MB_ERROR_NO_ERROR;
	status = MB_SUCCESS;

	/* return status */
	return(status);
}
/*--------------------------------------------------------------------*/
int mb_lever(int verbose,
		double sonar_offset_x,
		double sonar_offset_y,
		double sonar_offset_z,
		double nav_offset_x,
		double nav_offset_y,
		double nav_offset_z,
		double vru_offset_x,
		double vru_offset_y,
		double vru_offset_z,
		double vru_pitch,
		double vru_roll,
		double *lever_x,
		double *lever_y,
		double *lever_z,
		int *error)
{
	int	status = MB_SUCCESS;
	double	x, y, z;
	double	xx, yy, zz, r;
	double	pitch, roll;

	/* do lever calculation to find heave implied by roll and pitch
	   for a sonar displaced from the vru:
		x = r * COS(pitch) * COS(roll) 
		y = r * SIN(pitch)
		z = r * COS(pitch) * SIN(roll) */
	/* get net offset between sonar and vru */
	xx = sonar_offset_x - vru_offset_x;
	yy = sonar_offset_y - vru_offset_y;
	zz = sonar_offset_z - vru_offset_z;
	r = sqrt(xx * xx + yy * yy + zz * zz);
	
	/* lever arm only matters if offset is nonzero */
	if (r > 0.0)
	    {
	    /* get initial angles */
	    pitch = RTD * asin(yy / r);
	    if (cos(DTR * pitch) != 0.0)
		roll = RTD * acos(xx / (r * cos(DTR * pitch)));
	    else
		roll = 0.0;
		
  	    /* apply angle change */
	    pitch += vru_pitch;
	    roll += vru_roll;
	    
	    /* calculate new offsets */
	    x = r * cos(DTR * roll);
	    y = r * sin(DTR * pitch) * sin(DTR * roll);
	    z = r * cos(DTR * pitch) * sin(DTR * roll);
	    
	    /* get heave change due to lever arm */
	    *lever_z =  z - zz;
	    }
	else
	    {
	    *lever_z = 0.0;
	    }

	/* do lever calculation to find position shift implied by roll and pitch
	   for a sonar displaced from the nav sensor:
		x = r * COS(pitch) * COS(roll) 
		y = r * SIN(pitch)
		z = r * COS(pitch) * SIN(roll) */
	/* get net offset between sonar and nav sensor */
	xx = sonar_offset_x - nav_offset_x;
	yy = sonar_offset_y - nav_offset_y;
	zz = sonar_offset_z - nav_offset_z;
	r = sqrt(xx * xx + yy * yy + zz * zz);
	
	/* lever arm only matters if offset is nonzero */
	if (r > 0.0)
	    {
	    /* get initial angles */
	    pitch = RTD * asin(yy / r);
	    if (cos(DTR * pitch) != 0.0)
		roll = RTD * acos(xx / (r * cos(DTR * pitch)));
	    else
		roll = 0.0;
		
  	    /* apply angle change */
	    pitch += vru_pitch;
	    roll += vru_roll;
	    
	    /* calculate new offsets */
	    x = r * cos(DTR * roll);
	    y = r * sin(DTR * pitch) * sin(DTR * roll);
	    z = r * cos(DTR * pitch) * sin(DTR * roll);
	    
	    /* get position change due to lever arm */
	    *lever_x =  x - xx;
	    *lever_y =  y - yy;
	    }
	else
	    {
	    *lever_x = 0.0;
	    *lever_y = 0.0;
	    }

	/* assume success */
	*error = MB_ERROR_NO_ERROR;
	status = MB_SUCCESS;

	/* return status */
	return(status);
}
/*--------------------------------------------------------------------*/

