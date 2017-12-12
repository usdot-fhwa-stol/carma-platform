#include <boost/circular_buffer.hpp>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
namespace cav
{

/**
 * @brief this is a helper class to allow queing of messages to be processed in another thread without
 * blocking the enqueing thread
 * @tparam T - the type of message to be processed
 */
template<typename T>
class SynchronizedAsyncProcessQueue
{
    boost::circular_buffer<T> process_q_;
    std::mutex q_mutex_;
    std::condition_variable cv_;
    volatile bool running_;
    std::thread thread_;
    std::function<void (T&)> f_;
public:

    /**
     * @brief Constructs a SynchronizedAsyncProcessQueue
     * @param f - the function to be called to process messages
     * @param max_size - max size of the internal buffer. older messages are overwritten
     */
    SynchronizedAsyncProcessQueue(std::function<void (T&)> f,size_t max_size = 1000) :
            f_(f),
            running_(true),
            process_q_(max_size),
            thread_(std::bind(&SynchronizedAsyncProcessQueue::process,this))
    {

    }

    virtual ~SynchronizedAsyncProcessQueue()
    {
        running_ = false;
        cv_.notify_all();
        thread_.join();
    }

    /**
     * @brief Enque a message
     * @param t
     */
    void push(const T& t)
    {
        {
            std::unique_lock<std::mutex> lock(q_mutex_);
            process_q_.push_back(t);
        }

        cv_.notify_all();
    }

    /**
     * @brief Enque a message
     * @param t
     */
    void push(T&&t)
    {
        {
            std::unique_lock<std::mutex> lock(q_mutex_);
            process_q_.push_back(std::forward(t));
        }

        cv_.notify_all();
    }

private:

    /**
     * @brief The main process thread
     */
    void process()
    {
        while(running_)
        {
            std::unique_lock<std::mutex> lock(q_mutex_);
            if(process_q_.empty())
            {
                cv_.wait(lock,[this](){return !process_q_.empty() || !running_;});
            }

            if(!running_) break;
            T entry = process_q_.front();
            process_q_.pop_front();
            lock.unlock();
            f_(entry);
        }
    }
};
}//namespace cav
