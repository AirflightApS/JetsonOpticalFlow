#include "timing.h"

std::stack<clock_t> tictoc_stack;

void tic() {
    tictoc_stack.push(clock());
}

void toc() {
    std::cout << "Time elapsed: "
              << ((double)(clock() - tictoc_stack.top())) / CLOCKS_PER_SEC
              << std::endl;
    tictoc_stack.pop();
}

uint64_t micros(){

    static uint64_t start_time = 0;

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    uint64_t now_time = (now.tv_sec*1e6 + now.tv_nsec/1e3);

    if( start_time == 0 ){
        start_time = now_time;
    }

    return now_time - start_time;

}
