#ifndef THREAD_ARGS_H
#define THREAD_ARGS_H

#include <vector>             
#include <pthread.h>          
#include <utility>   

#include "route_node.h"
#include "thread_pool.h"
class ThreadArgs {
public:
    int id;
    void* router;          
    bool* success;         
    int max_run_time_per_bump;
    pthread_rwlock_t* layer_net_count_rwlock;
    ThreadPool* pool;
    ThreadArgs() {};
    ThreadArgs(int id_, void* router_, 
               bool* success_, int max_run_time_per_bump_,
               ThreadPool* pool_,
               pthread_rwlock_t* layer_net_count_rwlock_)
        : id(id_), router(router_),
        max_run_time_per_bump(max_run_time_per_bump_),
        pool(pool_),
        layer_net_count_rwlock(layer_net_count_rwlock_) {}
};

#endif