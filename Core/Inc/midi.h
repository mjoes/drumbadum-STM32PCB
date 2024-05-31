#include <cstdint>
#include <stdio.h>
#include "global.h"

#define MIDI_CLOCK     0xF8
#define MIDI_START     0xFA
#define MIDI_CONTINUE  0xFB
#define MIDI_STOP      0xFC

using namespace std;

class Midi {
public:
    Midi() = default;
    ~Midi() = default;

//    void CalculateBPM(uint8_t sync_type) {
//        uint32_t currentTick = HAL_GetTick();
//        uint32_t elapsedTime = currentTick - lastTick[sync_type];
//        lastTick[sync_type] = currentTick;
//        bpm_source[sync_type] = 60000 / elapsedTime;
//        reset_step_sample = true;
//    }

    void ProcessMidiByte(uint8_t rx_byte) {
        switch (rx_byte) {
            case MIDI_CLOCK:
                clockCount++;
                if (clockCount >= 24) {
                    clockCount = 0;
                    CalculateBPM(0);
                }
                break;
            case MIDI_START:
                if (run == false) {
                    step = 0;
                    step_sample = 0;
                    run = true;
                }
                break;
            case MIDI_CONTINUE:
                if (run == false){
                    step = stop_step;
                    step_sample = stop_sample;
                    run = true;
                }
                break;
            case MIDI_STOP:
                if (run == true){
                    stop_step = step;
                    stop_sample = step_sample;
                    run = false;
                }
                break;
            default:
                break; // Ignore other messages
        }
    }

//    uint8_t clk_source() {
//        uint32_t bpmTick = HAL_GetTick();
//        uint8_t clk_source;
//    	if (bpmTick - lastTick[0] < 1500){
//			clk_source = 0;
//            sync = true;
//		}
//		else if (bpmTick - lastTick[1] < 1500){
//			clk_source = 1;
//            sync = true;
//		}
//		else {
//			clk_source = 2;
//            sync = false;
//		}
//        return clk_source;
//    }

    uint8_t syncFlag() {
        return sync;
    }


private:
    uint8_t clockCount, sync;
    uint8_t stop_step = 0;
    uint16_t stop_sample = 0;
    uint32_t lastTick[2] = {};

};
