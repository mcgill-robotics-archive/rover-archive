#ifndef LINERANGER_COMMON_PERIODIC_THREAD_H
#define LINERANGER_COMMON_PERIODIC_THREAD_H

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

namespace lineranger
{
namespace common
{

/**
 * \brief Periodic thread
 *
 * Upon construction, this class spawns a new thread that will periodically
 * call a function encapsulated in a boost::function object. The thread is
 * cancelled in the destructor.
 *
 * On Linux, the implementation is mostly based on the RT PREEMPT How-To (see
 * https://rt.wiki.kernel.org/index.php/RT_PREEMPT_HOWTO). The scheduling
 * policy is always SCHED_FIFO.
 *
 * On Windows, we use standard and non-realtime boost functions for timing.
 *
 * The class supports several pre-defined priority values, which are in order:
 *
 *  - Real time
 *  - High
 *  - Above normal
 *  - Normal
 *  - Below normal
 *  - Idle
 *
 * <b> Basic usage </b>
 *
 * \code
 * #include <common/PeriodicThread.h>
 *
 * class MyPeriodicFunctionClass
 * {
 * public:
 *     void myFunc();
 * };
 *
 * MyPeriodicFunctionClass instance;
 * unsigned int cycleTime = 1000;
 * unsigned int startDelay = 1000;
 * int priority = PedioricFunction::getRealTimePriority();
 *
 * PeriodicThread(boost::bind(&PeriodicFunctionClass::myFunc, &instance), period, startDelay, priority) thread;
 *
 * \endcode
 */
class PeriodicThread
{
public:

    /// boost::function prototype of the periodic function
    typedef boost::function<void()> PeriodicFunction;

    /**
     * \brief Constructor.
     *
     * The thread is spawned in the constructor.
     *
     * \throw std::out_of_range if there's an invalid parameter.
     * \throw std::runtime_error if an error occurred during creation.
     *
     * \param fct boost::function object of the function to be periodically
     *      called.
     * \param cycleTime Desired cycle time in microseconds
     * \param startDelay Start delay in microseconds
     * \param priority Priority to be set.
     */
    PeriodicThread(PeriodicFunction fct, unsigned int cycleTime,
            unsigned int startDelay, int priority);

    /**
     * \brief Destructor.
     *
     * The thread is cancelled in the destructor.
     */
    virtual ~PeriodicThread();

    /**
     * \brief Get the cycle time.
     *
     * \return Cycle time in microseconds.
     */
    unsigned int getCycleTime() const;

    /**
     * \brief Get the start delay.
     *
     * \return Start delay in microseconds.
     */
    unsigned int getStartDelay() const;

    /**
     * \brief Get the thread priority.
     *
     * \return Thread priority.
     */
    int getPriority() const;

    /**
     * \brief Get the maximum priority value.
     *
     * \return Maximum priority value.
     */
    static int getMaxPriority();

    /**
     * \brief Get the minimum priority value.
     *
     * \return Minimum priority value.
     */
    static int getMinPriority();

    /**
     * \brief Get the real time priority value.
     *
     * \return Real time priority value.
     */
    static int getRealTimePriority();

    /**
     * \brief Get the high priority value.
     *
     * \return High priority value.
     */
    static int getHighPriority();

    /**
     * \brief Get the above normal priority value.
     *
     * \return Above normal priority value.
     */
    static int getAboveNormalPriority();

    /**
     * \brief Get the normal priority value.
     *
     * \return Normal priority value.
     */
    static int getNormalPriority();

    /**
     * \brief Get the below normal priority value.
     *
     * \return Below normal priority value.
     */
    static int getBelowNormalPriority();

    /**
     * \brief Get the idle priority value.
     *
     * \return Idle priority value.
     */
    static int getIdlePriority();

private:

    /// Internal thread function
    void threadFunction();

    /// boost::function object to be periodically called
    PeriodicFunction mPeriodicFunction;

    /// Thread object
    boost::scoped_ptr<boost::thread> mpThread;

    /// Flag to ask thread quit
    bool mQuit;
    
    /// Cycle time in microseconds
    unsigned int mCycleTime;

    /// Start delay in microseconds
    unsigned int mStartDelay;

    /// Thread priority
    int mPriority;

};

} // namespace common
} // namespace lineranger

#endif // LINERANGER_COMMON_PERIODIC_THREAD_H
