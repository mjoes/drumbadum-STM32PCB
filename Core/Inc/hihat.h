#include <cstdint>
#include <cmath>
#include <stdio.h>
#include <random>
#include <functional>
#include "envelopes.h"
#include "utils.h"

using namespace std;

struct HiHatSculpt {
    uint16_t velocity_;
};

class HiHat {
public:
    HiHat(
        uint16_t sample_rate,
        minstd_rand& gen)
        :
        sample_rate_(sample_rate),
        gen_(gen),
        dis(0, 200)

        {
            rel_pos_ = 0;
        }
    ~HiHat() {}

    void set_pattern(uint8_t pattern_nr, uint8_t random_pattern_nr, uint8_t randomness, bool accent) {
        if (accent == true) {
            randomness = 0;
        }
        set_decay(snd_random(patterns[pattern_nr][6],random_pattern_nr,6,randomness), bernoulli_draw(10));
        set_frequency(snd_random(patterns[pattern_nr][8],random_pattern_nr,8,randomness));
        set_bandwidth(snd_random(patterns[pattern_nr][9],random_pattern_nr,9,randomness));
        set_panning(randomness * 2, 50);
    }

    void set_decay(uint16_t decay, bool decay_type = 0) {
        decay_type_ = decay_type;
        length_decay_ = decay * sample_rate_ / 50;
        if (decay_type_ == 0) {
            lookup_table_ = exp_env;
        } else {
            lookup_table_ = log_env;
        }
    }

    void set_frequency(uint16_t frequency) {
        uint16_t frequency_map = frequency * 40 + 8000; //range from 8000-12000
        uint16_t index = (frequency_map * 1020 / sample_rate_) + 255;
        phi_ = sine[index]; // (65535.0 / 4.0)
    }

    void set_velocity(uint16_t velocity, bool accent) {
        if (accent == true) {
            HH.velocity_ = 1000;
        } else {
            HH.velocity_ = velocity;
        }
    }

    void set_bandwidth(uint16_t bandwidth) {
        // range is defined in the lambda lookup table from 300-3300
        // OLD --> HH.bandwidth_ = bandwidth * 3000 / 100 + 300; //range from 300-3300;
        uint16_t index = (bandwidth) * 256 / 100;
        lambda_ = lambda[index]; //
    }

    void set_start(uint8_t pattern_nr, uint8_t random_pattern_nr, uint8_t randomness, bool accent) {
        rel_pos_ = 0;
        running_ = true;
        set_velocity(800, accent);
        set_pattern(pattern_nr, random_pattern_nr, randomness, accent);

        a0 = 32767 / (1 + lambda_ / 255); // 65,535
        b1 = - lambda_ / 255 * phi_ * a0 / ((32757 / 2));
        b2 = a0 * lambda_ / 255 - a0;
    }

    void set_panning(uint8_t threshold, uint8_t amount) {
        // threshold sets how often we trigger random panning
        // amount sets how much panning
        int16_t pan_amount = 0;
        if (dis(gen_) > threshold) {
            pan_amount = (dis(gen_)) * amount / 100;
        }
        if (pan_amount > 0) { // pan right
            pan.pan_l = 90 - pan_amount;
            pan.pan_r = 100;
        } else { // pan_left
            pan.pan_l = 90;
            pan.pan_r = 100 + pan_amount;
        }
    }

    Out Process(uint8_t volume) {
        // Generate waveform sample
        if (running_ == false) {
            out.out_l = 0;
            out.out_r = 0;
            return out;
        }

        int32_t sample;
        sample = bp_filter_2(rand() % 65535 - 32767);
        sample = (sample * HH.velocity_) / 1000;
        interpolate_env_alt(&sample, rel_pos_, length_decay_, lookup_table_);
        int16_t output = (sample * volume) >> 7;

        rel_pos_ += 1;
        if (rel_pos_ >= length_decay_) {
            running_ = false;
        }
        out.out_l = output * pan.pan_l / 100;
        out.out_r = output * pan.pan_r / 100;
        return out;
    }

private:
    int16_t a0, b1, b2, lambda_, phi_;
    uint32_t rel_pos_, decay_, length_decay_;
    int32_t x_filter[2] = { 0, 0 };
    int32_t y_filter[2] = { 0, 0 };
    const uint16_t sample_rate_;
    const uint16_t* lookup_table_;
    bool running_, decay_type_;
    minstd_rand& gen_;
    uniform_int_distribution<int16_t> dis;
    HiHatSculpt HH;
    Out out;
    Panning pan;

    int32_t bp_filter_2(int32_t x_n) {
        int32_t filtered = a0 * x_n - a0 * x_filter[1] - b1 * y_filter[0] - b2 * y_filter[1];
        filtered >>= 15;
        y_filter[1] = y_filter[0];
        y_filter[0] = filtered;
        x_filter[1] = x_filter[0];
        x_filter[0] = x_n;
        return filtered;
    }
};
