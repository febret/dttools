/* standard include files */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

/* mb include file */
#include "mb_fbt.h"

/*--------------------------------------------------------------------*/
int mb_malloc(int verbose, size_t size, void **ptr, int *error)
{
	int	status = MB_SUCCESS;

	/* allocate memory */
	*ptr = NULL;
	if (size > 0)
		{
		if ((*ptr = (char *) malloc(size)) == NULL)
			{
			*error = MB_ERROR_MEMORY_FAIL;
			status = MB_FAILURE;
			}
		else
			{
			*error = MB_ERROR_NO_ERROR;
			status = MB_SUCCESS;
			}
		}
	else
		{
		*ptr = NULL;
		*error = MB_ERROR_NO_ERROR;
		status = MB_SUCCESS;
		}

	/* return status */
	return(status);
}
/*--------------------------------------------------------------------*/
int mb_free(int verbose, void **ptr, int *error)
{
	int	status = MB_SUCCESS;

	/* deallocate the memory if *ptr is not NULL */
	if (*ptr != NULL)
		{
		/* free the memory */
		free(*ptr);
		*ptr = NULL;
		}

	/* assume success */
	*error = MB_ERROR_NO_ERROR;
	status = MB_SUCCESS;

	/* return status */
	return(status);
}

