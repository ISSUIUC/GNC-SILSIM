#pragma once

#include <array>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <random>

class Perlin {
public:
    Perlin(unsigned int seed = 0) {
        std::iota(p_.begin(), p_.begin() + 256, 0);
        std::default_random_engine engine(seed);
        std::shuffle(p_.begin(), p_.begin() + 256, engine);
        for (int i = 0; i < 256; ++i)
            p_[256 + i] = p_[i];
    }

    double f(double x, double y, double z) const {
        int X = (int)std::floor(x) & 255;
        int Y = (int)std::floor(y) & 255;
        int Z = (int)std::floor(z) & 255;

        x -= std::floor(x);
        y -= std::floor(y);
        z -= std::floor(z);

        double u = fade(x);
        double v = fade(y);
        double w = fade(z);

        int A  = p_[X]     + Y;
        int AA = p_[A]     + Z;
        int AB = p_[A + 1] + Z;
        int B  = p_[X + 1] + Y;
        int BA = p_[B]     + Z;
        int BB = p_[B + 1] + Z;

        return lerp(w,
            lerp(v,
                lerp(u, grad(p_[AA],     x,     y,     z),
                        grad(p_[BA],     x - 1, y,     z)),
                lerp(u, grad(p_[AB],     x,     y - 1, z),
                        grad(p_[BB],     x - 1, y - 1, z))),
            lerp(v,
                lerp(u, grad(p_[AA + 1], x,     y,     z - 1),
                        grad(p_[BA + 1], x - 1, y,     z - 1)),
                lerp(u, grad(p_[AB + 1], x,     y - 1, z - 1),
                        grad(p_[BB + 1], x - 1, y - 1, z - 1))));
    }

private:
    std::array<int, 512> p_;

    static double fade(double t) {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    static double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }

    static double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
};