/*
 * eprom.c
 *
 *  Created on: Feb 17, 2020
 *      Author: saeedjassani
 */
#include <stdint.h>
#include "tm4c123gh6pm.h"

/*
 * EPROM data guide:
 *
 * 0 -> DHCP Mode
 * 1 -> IP
 * 2 -> GW
 * 3 -> DNS
 * 4 -> SN
 */


void initEeprom()
{
    SYSCTL_RCGCEEPROM_R = 1;
    _delay_cycles(3);
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

void writeEeprom(uint16_t add, uint32_t data)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    EEPROM_EERDWR_R = data;
    while (EEPROM_EEDONE_R & EEPROM_EEDONE_WORKING);
}

uint32_t readEeprom(uint16_t add)
{
    EEPROM_EEBLOCK_R = add >> 4;
    EEPROM_EEOFFSET_R = add & 0xF;
    return EEPROM_EERDWR_R;
}



