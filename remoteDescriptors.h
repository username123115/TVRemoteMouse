
#pragma once
#include <stdio.h>
#define totalButtons 8


uint8_t allDescriptors[totalButtons][2] = {
{3, 1}, //pan up
{0, 1}, //left click
{1, -5}, //left
{3, -1}, //pan down???
{2, -5}, //up
{1, 5}, //right
{0, 2}, //right click
{2, 5} //down
};

uint8_t commandIDs[totalButtons] = {15, 51, 60, 67, 76, 90, 96, 102};
uint8_t utilityID = 42;