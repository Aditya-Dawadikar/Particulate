#pragma once
#include <random>
#include <utility>

inline float randomFloat(float min, float max) {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

inline void hsvToRgb(float h, float s, float v, int& r, int& g, int& b) {
    float c = v * s;
    float x = c * (1 - std::fabs(fmod(h / 60.0, 2) - 1));
    float m = v - c;

    float r_, g_, b_;
    if (h < 60)       { r_ = c; g_ = x; b_ = 0; }
    else if (h < 120) { r_ = x; g_ = c; b_ = 0; }
    else if (h < 180) { r_ = 0; g_ = c; b_ = x; }
    else if (h < 240) { r_ = 0; g_ = x; b_ = c; }
    else if (h < 300) { r_ = x; g_ = 0; b_ = c; }
    else              { r_ = c; g_ = 0; b_ = x; }

    r = static_cast<int>((r_ + m) * 255);
    g = static_cast<int>((g_ + m) * 255);
    b = static_cast<int>((b_ + m) * 255);
}

struct pair_hash{
    template <class T1, class T2>
    std::size_t operator()(const std::pair<T1,T2>& p) const{
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1^(h2<<1);
    }
};