#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <vector>
#include <queue>
#include <pthread.h>
#include <iostream>
#include <utility>
#include <cstring>
#include "router.h" 
#include "thread_args.h" 

struct ThreadPool {
    std::mutex mutex;
    int next_net;
    const int num_nets;
    
    ThreadPool(int num_nets_) : next_net(0), num_nets(num_nets_) {}
    
    bool get_work(int& start_net, int& end_net) {
        const int CHUNK_SIZE = 1;  // Adjust chunk size as needed
        std::lock_guard<std::mutex> lock(mutex);
        
        if (next_net >= num_nets) {
            return false;
        }
        
        start_net = next_net;
        end_net = std::min(start_net + CHUNK_SIZE, num_nets);
        next_net = end_net;
        return true;
    }
};



#endif