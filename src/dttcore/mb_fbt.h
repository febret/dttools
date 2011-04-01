/* include this code only once */
#ifndef MBLGPL_HEADER_DEF
#define MBLGPL_HEADER_DEF

#include <math.h>

#define MB_FLAG_NULL 5

#define	MB_SSV_CORRECT	    1
#define	MB_SSV_INCORRECT    2


/* MBLGPL function boolean convention */
#define	MB_YES	1
#define	MB_NO	0

/* MBIO function status convention */
#define	MB_SUCCESS			1
#define	MB_FAILURE			0

/* MBIO function fatal error values */
#define	MB_ERROR_NO_ERROR		0
#define	MB_ERROR_MEMORY_FAIL	1
#define	MB_ERROR_BAD_PARAMETER	2
#define	MB_ERROR_EOF		4
#define	MB_ERROR_UNINTELLIGIBLE	-8

/* MBLGPL data type ("kind") convention */
#define	MB_DATA_NONE			0
#define	MB_DATA_DATA			1	/* general survey data */
#define	MB_DATA_COMMENT			2	/* general comment */

/* define some important sizes */
#define	MB_FBT_OLDHEADERSIZE	38
#define	MB_FBT_NEWHEADERSIZE	44
#define	MB_FBT_COMMENTSIZE		128

/* declare PI if needed */
#ifndef M_PI
#define	M_PI	3.14159265358979323846f
#endif

/* the natural log of 2 is always useful */
#define MB_LN_2        0.69314718056

/* multiply this by degrees to get radians */
#define DTR	0.01745329251994329500f

/* multiply this by radians to get degrees */
#define RTD	57.2957795130823230000f

/* min max round define */
#ifndef MIN
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif
#ifndef MAX
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif
#ifndef ROUND
#define	ROUND(X)	X < 0.0 ? ceil(X - 0.5) : floor(X + 0.5)
#endif


/* define byte swapping macros */
#define mb_swap_short(a) ( ((a & 0xff) << 8) | ((unsigned short)(a) >> 8) )
#define mb_swap_int(a) ( ((a) << 24) | \
                       (((a) << 8) & 0x00ff0000) | \
                       (((a) >> 8) & 0x0000ff00) | \
                        ((unsigned int)(a) >>24) )

#define mb_swap_long(a) ( ((a) << 56) | \
                       (((a) << 40) & 0x00ff000000000000) | \
                       (((a) << 24) & 0x0000ff0000000000) | \
                       (((a) <<  8) & 0x000000ff00000000) | \
                       (((a) >>  8) & 0x00000000ff000000) | \
                       (((a) >> 24) & 0x0000000000ff0000) | \
                       (((a) >> 40) & 0x000000000000ff00) | \
                        ((unsigned long)(a) >> 56)) 

// Function declarations.
int mb_malloc(int verbose, size_t size, void **ptr, int *error);
int mb_free(int verbose, void **ptr, int *error);
int mb_xyz_to_takeoff(int verbose, float x, float y, float z, float *theta, float *phi, int *error);

// Raytracing stuff.
int mb_rt_init(int verbose, int number_node, float *depth, float *velocity, void **modelptr, int *error);
int mb_rt_deall(int verbose, void **modelptr, int *error);
int mb_rt(int verbose, void *modelptr, double source_depth, double source_angle, double end_time, int ssv_mode, double surface_vel, double null_angle, 
	int nplot_max, int *nplot, double *xplot, double *zplot, double *x, double *z, double *travel_time, int *ray_stat, int *error);
int mb_rt_circular(int verbose, int *error);
int mb_rt_quad1(int verbose, int *error);
int mb_rt_quad2(int verbose, int *error);
int mb_rt_quad3(int verbose, int *error);
int mb_rt_quad4(int verbose, int *error);
int mb_rt_get_depth(int verbose, double beta, int dir_sign, int turn_sign, double *depth, int *error);
int mb_rt_plot_circular(int verbose, int *error);
int mb_rt_line(int verbose, int *error);
int mb_rt_vertical(int verbose, int *error);

#endif
