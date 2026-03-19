#ifndef VI_TREADS
#define VI_TREADS

#include <thread>
#include <queue>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <future>
#include <vector>

namespace vi_treads {

    class TreadPool {
        public:
            TreadPool() = delete;
            explicit TreadPool(uint num_tread);
            ~TreadPool();

            template<typename Func, typename... Args>
                void execute(Func&& func, Args&&... args);

            template<typename Func, typename... Args>
                auto submit(Func&& func, Args&&... args) -> std::future<decltype(func(args...))>;


        private:
            mutable std::mutex mt;
            std::condition_variable data_condition;
            std::vector<std::thread> workers;
            std::queue<std::function<void()>> tasks; 
            bool stop = false;
    };




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
#endif