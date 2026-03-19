#include <threadPool.hpp>

namespace vi_treads {

    ThreadPool::ThreadPool(uint num_tread) {
        for(uint i = 0; i < num_tread; ++i)
            workers.emplace_back([this]{
                while(true)
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lk(mt);
                        data_condition.wait(lk, [this]{
                            return stop || !tasks.empty();
                        });
                        if (stop && tasks.empty()) return;
                        task = std::move(tasks.front());
                        tasks.pop();
                    }
                    task();
                }
            });
    }

    ThreadPool::~ThreadPool() {
        {
            std::lock_guard<std::mutex> lk(mt);
            stop = true;
        }

        data_condition.notify_all();

        for(auto& worker: workers)
            worker.join();
    }
}