#include <treadPool.hpp>

namespace vi_treads {

    TreadPool::TreadPool(uint num_tread) {
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

    TreadPool::~TreadPool() {
        {
            std::lock_guard<std::mutex> lk(mt);
            stop = true;
        }

        data_condition.notify_all();

        for(auto& worker: workers)
            worker.join();
    }

    //для void function
    template<typename Func, typename... Args>
        void TreadPool::execute(Func&& func, Args&&... args)
        {
            {
            std::lock_guard<std::mutex> lk(mt);
            tasks.push(std::bind(std::forward<Func>(func), std::forward<Args>(args)...));
            }
            data_condition.notify_one();
        }

    //для функций с возращаемым значением
    template<typename Func, typename... Args>
        auto TreadPool::submit(Func&& func, Args&&... args) -> std::future<decltype(func(args...))>
        {
            using return_type = decltype(func(args...));

            auto task = std::make_shared<std::packaged_task<return_type()>>(
                std::bind(std::forward<Func>(func), std::forward<Args>(args)...)
            );

            std::future<return_type> result = task->get_future();

            {
                std::lock_guard<std::mutex> lk(mt);
                tasks.push([task]{ (*task)(); });
            }

            data_condition.notify_one();
            return result;
        }

}