#ifndef __BERLIN_NOISE__
#define __BERLIN_NOISE__

#include <cmath>
#include <array>
#include <random>
#include <algorithm>

class PerlinNoise {
public:

    PerlinNoise() {

        std::iota(p.begin(), p.end(), 0);
        std::random_device rd;
        std::mt19937 generator(rd());
        std::shuffle(p.begin(), p.end(), generator);


        for (int i = 0; i < 256; ++i) {
            p[256 + i] = p[i];
        }
    }

    double noise(double x, double y, double z) const {
 
        int X = (int)floor(x) & 255;
        int Y = (int)floor(y) & 255;
        int Z = (int)floor(z) & 255;

 
        x -= floor(x);
        y -= floor(y);
        z -= floor(z);

 
        double u = fade(x);
        double v = fade(y);
        double w = fade(z);


        int A = p[X] + Y;
        int AA = p[A & 255] + Z;
        int AB = p[(A + 1) & 255] + Z;
        int B = p[(X + 1) & 255] + Y;
        int BA = p[B & 255] + Z;
        int BB = p[(B + 1) & 255] + Z;

  
        double res = lerp(w, lerp(v, lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)), lerp(u, grad(p[AB], x, y - 1, z), grad(p[BB], x - 1, y - 1, z))),
                          lerp(v, lerp(u, grad(p[AA + 1], x, y, z - 1), grad(p[BA + 1], x - 1, y, z - 1)), lerp(u, grad(p[AB + 1], x, y - 1, z - 1), grad(p[BB + 1], x - 1, y - 1, z - 1))));
        return (res + 1.0) / 2.0; 
    }

private:
    std::array<int, 512> p;

    static double fade(double t) {

        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    static double lerp(double t, double a, double b) {
        return a + t * (b - a);
    }

    static double grad(int hash, double x, double y, double z) {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
};
#endif