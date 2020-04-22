/*
 * eprom.h
 *
 *  Created on: Feb 17, 2020
 *      Author: saeedjassani
 */

#ifndef EPROM_H_
#define EPROM_H_

void initEeprom();
void writeEeprom(uint16_t add, uint32_t data);
uint32_t readEeprom(uint16_t add);


#endif /* EPROM_H_ */
