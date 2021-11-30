#ifndef TIMING_H
#define TIMING_H

#include <iostream>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <stack>
#include <ctime>

uint64_t micros();

// Usefull to measure time of a task
void tic();
void toc();

#endif // TIMING_H
