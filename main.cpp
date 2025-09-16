#include <pybind11/pybind11.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

#include <atomic>

int add(int a, int b) {
    return a + b;
}

long parallel_sum(int count) {
    std::atomic<long> total{0};
    if(count < 0) {
        throw std::invalid_argument("count must be non-negative");
    }

    tbb::parallel_for(tbb::blocked_range<int>(0, count), [&](const tbb::blocked_range<int>& r) {
        long local = 0;
        for(int i = r.begin(); i != r.end(); ++i) {
            local += i;
        }
        total.fetch_add(local, std::memory_order_relaxed);
    });

    return total.load(std::memory_order_relaxed);
}

PYBIND11_MODULE(hinacloth, m) {
    m.doc() = "Minimal pybind11 test module for HinaCloth";
    m.def("add", &add, "Return the sum of two integers");
    m.def("parallel_sum", &parallel_sum, "Sum integers [0, count) using oneTBB parallel_for");
}