#pragma once

#include <array>
#include <cstring>

template<typename T, size_t Size>
class Buffer {
private:
    std::array<T, Size> data;
    size_t head = 0;
    size_t count = 0;

public:
    Buffer() = default;
    
    void push(const T& value) {
        data[head] = value;
        head = (head + 1) % Size;
        if (count < Size) {
            count++;
        }
    }
    
    T getRecent() const {
        if (count == 0) return T{};
        size_t index = (head - 1 + Size) % Size;
        return data[index];
    }
    
    T getRecentUnsync() const {
        return getRecent();
    }
    
    void readSlice(T* dest, size_t start, size_t length) const {
        size_t actualLength = std::min(length, count);
        for (size_t i = 0; i < actualLength; i++) {
            size_t index = (head - count + start + i + Size) % Size;
            dest[i] = data[index];
        }
        for (size_t i = actualLength; i < length; i++) {
            dest[i] = T{};
        }
    }
    
    size_t size() const {
        return count;
    }
    
    bool empty() const {
        return count == 0;
    }
    
    void clear() {
        head = 0;
        count = 0;
    }
};
