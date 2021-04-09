#ifndef _NODE_TYPES_H
#define _NODE_TYPES_H

#include <stdint.h>

#define STRUCT_TYPE_RADIOSTROM_3000 0x33

typedef struct radiostrom3000_t
{
    uint8_t stype = STRUCT_TYPE_RADIOSTROM_3000;

     // battery in range [0 .. 1]
    uint8_t battery = 0;

    //! voltage in mV
    uint16_t voltage = 0;

    //! current in mA
    uint16_t current = 0;

    //! power in mW
    uint16_t power = 0;

} radiostrom3000_t;

#endif
