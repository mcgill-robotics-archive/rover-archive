#include "PeriodicThread.h"


#include <stdio.h>

using namespace lineranger;
using namespace lineranger::common;

#ifdef WIN32
    #include <windows.h>
    #include <boost/date_time/posix_time/posix_time.hpp>
#else
    #if !(_XOPEN_SOURCE >= 600 || _POSIX_C_SOURCE >= 200112L)
    #error "clock_nanosleep NOT supported"
    #endif
    #include <sched.h>
    #include <unistd.h>
    #include <sys/utsname.h>
#endif

#define NSEC_PER_USEC  (1000)       // The number of nanoseconds per microseconds.
#define NSEC_PER_SEC   (1000000000) // The number of nanoseconds per seconds.


#ifndef WIN32

// Add a time interval (microseconds) to a timespec
static void addTimeInterval(timespec& t, unsigned int interval)
{
    t.tv_nsec += static_cast<int>(interval*NSEC_PER_USEC);

    while(t.tv_nsec >= NSEC_PER_SEC)
    {
        t.tv_nsec -= NSEC_PER_SEC;
        t.tv_sec++;
    }
}

static bool isLinuxRT()
{
    struct utsname u;
    bool crit1;
    int crit2 = 0;
    FILE *fd;

    uname(&u);
    crit1 = (strcasestr(u.version, "PREEMPT RT") != NULL);

    if ((fd = fopen("/sys/kernel/realtime","r")) != NULL)
    {
        int flag;
        crit2 = ((fscanf(fd, "%d", &flag) == 1) && (flag == 1));
        fclose(fd);
    }

    return (crit1 && crit2);
}

#endif // WIN32

PeriodicThread::PeriodicThread(PeriodicFunction fct, unsigned int cycleTime,
    unsigned int startDelay, int priority) :
        mPeriodicFunction(fct),
        mQuit(false),
        mCycleTime(cycleTime),
        mStartDelay(startDelay),
        mPriority(priority)
{
    if(mCycleTime <= 0)
    {
        throw std::out_of_range("Thread cycle time must be positive.");
    }

    if(startDelay < 0)
    {
        throw std::out_of_range("Start delay can't be negative.");
    }

    if((mPriority > getMaxPriority()) || (mPriority < getMinPriority()))
    {
        throw std::out_of_range("Thread priority out of range.");
    }

    mpThread.reset(new boost::thread(&PeriodicThread::threadFunction, this));

    if(!mpThread)
        throw std::runtime_error("Failed to create new thread.");
}

PeriodicThread::~PeriodicThread()
{
    /// Ask the thread to quit and wait indefinitely for join
    mQuit = true;
    mpThread->join();
}

void PeriodicThread::threadFunction()
{
#ifdef WIN32

    SetThreadPriority(mpThread->native_handle(), mPriority);

    boost::this_thread::sleep(boost::posix_time::microseconds(mStartDelay));

    while(!mQuit)
    {
        mPeriodicFunction();
        boost::this_thread::sleep(boost::posix_time::microseconds(mCycleTime));
    }

#else

    struct timespec t;
    struct sched_param param;

    if(isLinuxRT())
    {
//        LR_LOG_DEBUG() << "This linux kernel is RT-Preempt.";
    }
    else
    {
//        LR_LOG_DEBUG() << "This linux kernel is NOT RT-Preempt.";
    }

    // Declare ourself as a real time task
    param.sched_priority = mPriority;
    if(sched_setscheduler(0, SCHED_FIFO, &param) == -1)
    {
//        LR_LOG_ERROR() << "Failed to set scheduling policy: " << std::strerror(errno);
    }

    clock_gettime(CLOCK_MONOTONIC ,&t);
    addTimeInterval(t, mStartDelay);

    while(!mQuit)
    {
        // Do work
        mPeriodicFunction();

        // Calculate next shot
        addTimeInterval(t, mCycleTime);

        // wait until next shot
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
    }
#endif
}

unsigned int PeriodicThread::getCycleTime() const
{
    return mCycleTime;
}

unsigned int PeriodicThread::getStartDelay() const
{
    return mStartDelay;
}

int PeriodicThread::getPriority() const
{
    return mPriority;
}

int PeriodicThread::getMaxPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_TIME_CRITICAL;
#else
    return sched_get_priority_max(SCHED_FIFO);
#endif
}

int PeriodicThread::getMinPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_IDLE;
#else
    return sched_get_priority_min(SCHED_FIFO);
#endif
}

int PeriodicThread::getRealTimePriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_TIME_CRITICAL;
#else
    return 49;
#endif
}

int PeriodicThread::getHighPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_HIGHEST;
#else
    return 40;
#endif
}

int PeriodicThread::getAboveNormalPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_ABOVE_NORMAL;
#else
    return 30;
#endif
}

int PeriodicThread::getNormalPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_NORMAL;
#else
    return 20;
#endif
}

int PeriodicThread::getBelowNormalPriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_BELOW_NORMAL;
#else
    return 10;
#endif
}

int PeriodicThread::getIdlePriority()
{
#ifdef WIN32
    return THREAD_PRIORITY_LOWEST;
#else
    return 1;
#endif
}
