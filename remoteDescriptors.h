
#pragma once
#include <stdio.h>
#define totalButtons 4


uint8_t allDescriptors[totalButtons][5] = {
{0, -5, 0, 0, 0}, //left
{0, 0, -5, 0, 0}, //up
{0, 5, 0, 0, 0}, //right
{0, 0, 5, 0, 0} //down
};

uint8_t commandIDs[totalButtons] = {60, 76, 90, 102};