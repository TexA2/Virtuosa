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
}
#endif