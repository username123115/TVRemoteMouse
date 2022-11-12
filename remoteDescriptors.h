
#pragma once
#include <stdio.h>
#define totalButtons 4

typedef struct remoteDescriptor{
    char id;
    uint8_t effects[5];
} remoteDescriptor;
uint8_t left[] = {0, -5, 0, 0, 0};
uint8_t up[] = {0, 0, -5, 0, 0};
uint8_t right[] = {0, 5, 0, 0, 0};
uint8_t down[] = {0, 0, 5, 0, 0};
remoteDescriptor rLeft = (remoteDescriptor) {60, (uint8_t) left};
remoteDescriptor rUp = (remoteDescriptor) {76, (uint8_t) up};
remoteDescriptor rRight = (remoteDescriptor) {90, (uint8_t) right};
remoteDescriptor rDown = (remoteDescriptor) {102, (uint8_t) down};
remoteDescriptor allDescriptors[totalButtons] = {rLeft, rUp, rRight, rDown};