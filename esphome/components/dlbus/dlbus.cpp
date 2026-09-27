#include "esphome/core/log.h"
#include "dlbus.h"
#include <cstdint>

namespace esphome::dlbus {

static const char *TAG = "dlbus.component";

void IRAM_ATTR HOT DlBusStore::gpio_intr(DlBusStore *arg) {
  const bool new_level = arg->pin_.digital_read();
  const uint32_t now = micros();
  const uint32_t pulse_width = now - arg->last_val_edge_;

  switch (arg->state_) {
    case SYNC:
      // if we are waiting for a SYNC and are on a rising edge...
      if (new_level) {
        arg->last_val_edge_ = now;

        // ...and the measured pulse is roughly the same as the ones before...
        if (arg->on_next_clock_(pulse_width)) {
          arg->bits_read_++;
        } else {
          arg->bits_read_= 1;
          arg->clock_ = pulse_width;
        }

        if (arg->bits_read_ >= 16) {
          arg->state_ = START_BIT;
          arg->bits_read_ = 0;
        }
      }
      break;

    case START_BIT:
      // if the frame was a sensor request and a sensor device answers (which happens without a SYNC)
      // the start bit doesn't follow immedately (there's a minimum delay of 20ms)
      if (arg->on_clock_(pulse_width)) {
        if (!new_level) { // this means another start bit
          arg->last_val_edge_ = now;
          arg->state_ = DATA_BIT;
        } else {
          // here we assume that we just witnessed the first sync bit, and therefore finish the current message
          arg->finish_frame_();
          arg->sync_();
          arg->bits_read_ = 1;
        }
      }
      break;

    case STOP_BIT:
      if (arg->on_next_clock_(pulse_width)) {
        if (new_level) {
          arg->last_val_edge_ = now;
          arg->state_ = START_BIT;
        } else {
          // this shouldn't normally happen - force a sync
          arg->sync_();
        }
      }
      break;

    case DATA_BIT:
      if (arg->on_next_clock_(pulse_width)) {
        arg->last_val_edge_ = now;

        const auto byte_offset = arg->bits_read_ / 8;
        const auto bit_offset = arg->bits_read_ % 8;      

        if (byte_offset >= arg->buffers_[arg->active_buffer_].data.size()) {
          // we've read to many bits for the buffer :(
          arg->sync_();
        } else {
          uint8_t &byte = arg->buffers_[arg->active_buffer_].data[byte_offset];
          uint8_t mask = 1 << bit_offset;
          byte = (byte & ~mask) | (new_level ? mask : 0);

          arg->bits_read_++;
          if (arg->bits_read_ % 8 == 0) {
            arg->state_ = STOP_BIT;
          }
        }
      }
      break;    
  }
  // if we're way past the cycle – force a sync
  // we have to be over by at least 100ms, because that is the timeout for a sensor device answering the contoller
  if (pulse_width > 150000) {
    arg->sync_();
  }
}

void DlBus::loop() {
  // Tasks here will be performed at every call of the main application loop.
  // Note: code here MUST NOT BLOCK (see below)

  // ESP_LOGI(TAG, "state: %d, prev_byets: %d", this->store_.state_, this->store_.prev_bytes_read_);
  // ESP_LOGI(TAG, "last_pulse_w: %ld %s", this->store_.last_pulse_width, this->store_.last_level ? "rising" : "falling");
  //
  const DlBusFrame *frame;
  if (this->store_.get_frame(frame)) {
    ESP_LOGI(TAG, "read frame: %d bytes, first byte: %x", frame->length, frame->data[0]);
  }
}

void DlBus::dump_config(){
  ESP_LOGCONFIG(TAG, "DL-Bus");
  LOG_PIN("  Pin: ", this->pin_);
}

}  // namespace esphome::dlbus
