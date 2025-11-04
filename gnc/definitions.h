#pragma once

#include <iostream>
#include <thread>
#include <chrono>

#define THREAD_SLEEP(ms) std::this_thread::sleep_for(std::chrono::milliseconds(ms))

#define pdTICKS_TO_MS(ticks) (ticks)
#define xTaskGetTickCount() (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count())
class MockSerial {
public:
    template<typename T>
    void println(T value) {
        std::cout << value << std::endl;
    }
};

MockSerial Serial;
