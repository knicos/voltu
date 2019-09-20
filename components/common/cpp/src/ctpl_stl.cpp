#ifdef WIN32
#include <Ws2tcpip.h>
#include <windows.h>
#endif

#include <ctpl_stl.h>

void ctpl::thread_pool::set_thread(int i) {
    std::shared_ptr<std::atomic<bool>> flag(this->flags[i]); // a copy of the shared ptr to the flag
    auto f = [this, i, flag/* a copy of the shared ptr to the flag */]() {
        std::atomic<bool> & _flag = *flag;
        std::function<void(int id)> * _f;
        bool isPop = this->q.pop(_f);
        while (true) {
            while (isPop) {  // if there is anything in the queue
                std::unique_ptr<std::function<void(int id)>> func(_f); // at return, delete the function even if an exception occurred
                (*_f)(i);
                if (_flag)
                    return;  // the thread is wanted to stop, return even if the queue is not empty yet
                else
                    isPop = this->q.pop(_f);
            }
            // the queue is empty here, wait for the next command
            std::unique_lock<std::mutex> lock(this->mutex);
            ++this->nWaiting;
            this->cv.wait(lock, [this, &_f, &isPop, &_flag](){ isPop = this->q.pop(_f); return isPop || this->isDone || _flag; });
            --this->nWaiting;
            if (!isPop)
                return;  // if the queue is empty and this->isDone == true or *flag then return
        }
    };
    this->threads[i].reset(new std::thread(f)); // compiler may not support std::make_unique()

	// For excess threads, ensure they only operate if needed.
	/*if (i >= std::thread::hardware_concurrency()-1) {
		#ifndef WIN32
		sched_param p;
		p.sched_priority = sched_get_priority_min(SCHED_FIFO);
		pthread_setschedparam(threads[i]->native_handle(), SCHED_FIFO, &p);
		#endif
	} else {
		#ifndef WIN32
		sched_param p;
		p.sched_priority = sched_get_priority_max(SCHED_FIFO);
		pthread_setschedparam(threads[i]->native_handle(), SCHED_FIFO, &p);
		#endif
	}*/

	/*
    #ifdef WIN32
    SetThreadAffinityMask(this->threads[i]->native_handle(), 1 << i);
    #else
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(i, &cpus);
    pthread_setaffinity_np(this->threads[i]->native_handle(), sizeof(cpus), &cpus);
    #endif
	*/
}
