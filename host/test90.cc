#include <iostream>
#include <mutex>
#include <condition_variable>
#include <thread>

std::mutex mutex_;
std::condition_variable condVar;
bool dataReady = false;

void Trigger_Thread(int nr)
{
    std::cout << nr << ". Trigger_Thread startet" << std::endl;
    while(true)
    {
        std::cin.get();
        {
            std::lock_guard<std::mutex> lck(mutex_);
            dataReady = true;
        }
        std::cout << nr << "xxx" << std::endl;
        condVar.notify_one();
    }
}

void Worker_Thread(int nr)
{
    std::cout << nr << ". Worker_Thread startet" << std::endl;
    while(true)
    {
        {
            std::unique_lock<std::mutex> lck(mutex_);
//            condVar.wait(lck,[&]{return dataReady;});
            condVar.wait(lck);
        }
        // work work work
        std::cout << nr << " Work done." << std::endl;
    }
}

int main()
{
    std::thread thread1(Trigger_Thread,1);

    for(int i = 1; i <= 5; i++)
    {
        std::thread(Worker_Thread,i).detach();
    }

    thread1.join();
}