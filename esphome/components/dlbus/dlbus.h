#pragma once

#include <cstddef>
#include <cstdint>
#include "esphome/core/component.h"
#include "esphome/core/hal.h"

namespace esphome::dlbus {

static size_t constexpr BUFFER_LEN = 64; 
  
enum DlBusState {
  SYNC,
  START_BIT,
  DATA_BIT,
  STOP_BIT,
};

struct DlBusFrame {
  std::array<uint8_t, 64> data{};
  size_t length = 0;
};

class DlBusStore {
  public:
    void setup(InternalGPIOPin *pin) {
      pin->setup();
      this->pin_ = pin->to_isr();
      pin->attach_interrupt(&DlBusStore::gpio_intr, this, gpio::INTERRUPT_ANY_EDGE);
    }
    static void gpio_intr(DlBusStore *arg);

    bool get_frame(const DlBusFrame *&out) {
      if (!this->frame_ready_) return false;
      // use the currently inactive buffer
      out = &this->buffers_[this->active_buffer_ ^ 1];
      this->frame_ready_ = false;
      return true;
    }
    volatile DlBusState state_ = SYNC;
    size_t prev_bytes_read_ = 0;

  protected:
    ISRInternalGPIOPin pin_;

    uint32_t last_val_edge_ = 0;    
    uint32_t clock_ = 0;

    size_t bits_read_ = 0;

    volatile bool frame_ready_ = false;
    size_t active_buffer_ = 0;
    std::array<DlBusFrame, 2> buffers_{};
    
    bool on_next_clock_(uint32_t pulse_width) {
      return ((this->clock_ - this->clock_ / 4) < pulse_width &&
               pulse_width < (this->clock_ + this->clock_ / 4));
    }

    bool on_clock_(uint32_t pulse_width) {
      uint32_t inter_clock_width = pulse_width % this->clock_;
      return ((inter_clock_width < this->clock_ / 4) ||
              (inter_clock_width > this->clock_ + this->clock_ / 4));
    }

    void sync_() { this->state_ = SYNC; this->bits_read_ = 0; }
    void finish_frame_() {
      // write length in bytes to frame
      this->buffers_[this->active_buffer_].length = this->bits_read_ / 8;
      this->active_buffer_ ^= 1;
      this->frame_ready_ = true;
    }
};

class DlBus final : public Component {
public:
  void set_pin(InternalGPIOPin *pin) { this->pin_ =pin; }

  void setup() override { this->store_.setup(this->pin_); }
  void loop() override;
  void dump_config() override;

protected:
  DlBusStore store_;
  InternalGPIOPin *pin_;
};

}  // namespace esphome::dlbus

