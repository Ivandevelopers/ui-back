#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <mutex>
#include <condition_variable>

class ThreadTimer
{
private:
    bool active;
	int reties = 3;
    std::mutex m;
    std::condition_variable cv;
    std::thread timerThread;

    void threadProcessInterval(std::function<void()> handler,int timeout) {
		while(active && reties-- > 0) {
			std::unique_lock<std::mutex> lock(m);
			auto status = cv.wait_for(lock,std::chrono::milliseconds(timeout));
			if (status == std::cv_status::timeout && active) {
				if (handler != nullptr) {
					handler();
				}
			}
		}
	}

    void threadProcessTimeout(std::function<void()> handler, int timeout)
    {
        std::unique_lock<std::mutex> lock(m);
        auto status = cv.wait_for(lock, std::chrono::milliseconds(timeout));
        if (status == std::cv_status::timeout && active) {
            if (handler != nullptr) {
                handler();
            }
            active = false;
        }
    
    }

public:
    ThreadTimer() : active(false) {}

    void setReties(int number_of_retries) { reties = number_of_retries; }

    void setInterval(std::function<void()> handler, int timer) {
		clear();
		active = true;
        timerThread = std::thread(&ThreadTimer::threadProcessInterval, this, handler, timer);
		timerThread.detach();
	}

    void setTimeout(std::function<void()> handler, int timer)
    {
        clear();
        active = true;
        timerThread = std::thread(&ThreadTimer::threadProcessTimeout, this, handler, timer);
        timerThread.detach();
    }



    void clear()
    {
        std::lock_guard<std::mutex> lock(m);
        active = false;
        cv.notify_all();
    }

    ~ThreadTimer()
    {
        clear();
        if (timerThread.joinable()) {
            timerThread.join();
        }
    }
};