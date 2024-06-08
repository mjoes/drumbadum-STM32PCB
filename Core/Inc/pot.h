#include <cstdint>

using namespace std;

class Pot {
public:
    Pot() = default;
    ~Pot() = default;

    void set_pot_val(uint16_t read_val) { 
        pot_val_ = 4096 - read_val;
    }

    void set_init_val(uint16_t pot_pos) { 
        pot[1] = pot_pos;
    }

    void mode_change(bool new_mode) {
        current_mode_ = new_mode;
        old_mode_ = !current_mode_;
    	if (active_ == true) {
    		pot[old_mode_] = pot_val_;
    	}

        conditionFlag_ = (pot_val_ >= pot[current_mode_]);
        active_ = false;
    }

    uint16_t Process() {
        if (!active_) {
            if ((pot_val_ > pot[current_mode_] - 30 && conditionFlag_ == 0) ||
                (pot_val_ < pot[current_mode_] + 30 && conditionFlag_ == 1)) {
                active_ = true;
            }
        }
        uint16_t out = (active_) ? pot_val_ : pot[current_mode_];
        return out;
    }

private:
    bool current_mode_, old_mode_, conditionFlag_;
    bool active_ = true;
    uint16_t pot_val_;
    uint16_t pot[2] = { 2000, 2000 };
};
