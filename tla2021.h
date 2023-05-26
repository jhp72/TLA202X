#ifndef __TLA2021_H__
#define __TLA2021_H__
/*
 * tla2021.h
 *
 *  Created on: 2023.04.17.
 *      Author: jaehong park (jaehong1972@gmail.com)
 */

#include "nx_config.h"


/*
    Public function exposed to app_main()
*/
void tla2021_init(void);
void tla2021_process(void);
void cli_dev_tla2021_rst(void);


#endif //__TLA2021_H__
