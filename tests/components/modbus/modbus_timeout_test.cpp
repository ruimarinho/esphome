#include "common.h"

namespace esphome::modbus::testing {

class ModbusTimeoutTest : public ::testing::Test {
 protected:
  TestUARTComponent uart_;
  Modbus modbus_;
  TestModbusDevice device_;

  void SetUp() override {
    modbus_.set_uart_parent(&uart_);
    modbus_.set_role(ModbusRole::CLIENT);
    modbus_.set_send_wait_time(20);
    modbus_.set_turnaround_time(0);
    modbus_.set_disable_crc(false);

    device_.set_parent(&modbus_);
    device_.set_address(0x01);
    modbus_.register_device(&device_);

    modbus_.setup();
  }

  void queue_read(uint16_t reg_addr, uint16_t count = 1) { device_.send(0x04, reg_addr, count); }

  void pump_until_sent(int max_loops = 100) {
    uart_.clear_written();
    for (int i = 0; i < max_loops; i++) {
      modbus_.loop();
      if (!uart_.written_data.empty()) {
        return;
      }
      sleep_ms(1);
    }
  }

  void pump_loops(int count, uint32_t delay_ms = 1) {
    for (int i = 0; i < count; i++) {
      modbus_.loop();
      if (delay_ms > 0) {
        sleep_ms(delay_ms);
      }
    }
  }
};

TEST_F(ModbusTimeoutTest, NormalResponseWithinTimeout) {
  queue_read(0x0016, 1);
  pump_until_sent();
  ASSERT_FALSE(uart_.written_data.empty());

  uart_.inject_rx(make_response(0x01, 0x04, {0x00, 0x04, 0xD2, 0x00}));
  modbus_.loop();

  ASSERT_EQ(device_.received_data.size(), 1);
  EXPECT_EQ(device_.received_data[0].size(), 4);
}

TEST_F(ModbusTimeoutTest, LateResponseDoesNotGetMisattributedToNextCommand) {
  queue_read(0x000C, 1);
  queue_read(0x0079, 1);

  pump_until_sent();
  ASSERT_FALSE(uart_.written_data.empty());

  sleep_ms(30);
  modbus_.loop();

  uart_.inject_rx(make_response(0x01, 0x04, {0x00, 0x00, 0x1A, 0xF4}));
  modbus_.loop();

  sleep_ms(30);
  pump_until_sent(50);

  uart_.inject_rx(make_response(0x01, 0x04, {0x00, 0x00, 0x01, 0xFB}));
  modbus_.loop();

  bool found_contract_value = false;
  for (const auto &data : device_.received_data) {
    if (data.size() >= 4) {
      uint32_t value =
          (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
          (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
      if (value == 0x00001AF4) {
        found_contract_value = true;
      }
    }
  }

  EXPECT_FALSE(found_contract_value);
}

TEST_F(ModbusTimeoutTest, SplitLateResponseIsIgnoredDuringQuarantine) {
  queue_read(0x000C, 1);
  queue_read(0x0079, 1);

  pump_until_sent();

  sleep_ms(30);
  modbus_.loop();

  const std::vector<uint8_t> full_response = make_response(0x01, 0x04, {0x00, 0x00, 0x1A, 0xF4});
  uart_.inject_rx(std::vector<uint8_t>(full_response.begin(), full_response.begin() + 4));
  modbus_.loop();
  sleep_ms(2);

  uart_.inject_rx(std::vector<uint8_t>(full_response.begin() + 4, full_response.end()));
  modbus_.loop();

  sleep_ms(30);
  pump_until_sent(50);

  uart_.inject_rx(make_response(0x01, 0x04, {0x00, 0x00, 0x01, 0xFB}));
  modbus_.loop();

  bool found_contract_value = false;
  for (const auto &data : device_.received_data) {
    if (data.size() >= 4) {
      uint32_t value =
          (static_cast<uint32_t>(data[0]) << 24) | (static_cast<uint32_t>(data[1]) << 16) |
          (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
      if (value == 0x00001AF4) {
        found_contract_value = true;
      }
    }
  }

  EXPECT_FALSE(found_contract_value);
}

TEST_F(ModbusTimeoutTest, RepeatedTimeoutsDoNotDeadlockQueue) {
  for (int i = 0; i < 5; i++) {
    queue_read(0x0016 + i, 1);
  }

  for (int i = 0; i < 5; i++) {
    pump_until_sent(100);
    sleep_ms(30);
    pump_loops(5, 5);
  }

  EXPECT_TRUE(modbus_.tx_buffer_empty());
}

TEST_F(ModbusTimeoutTest, ResponseJustUnderTimeoutStillDispatchesNormally) {
  queue_read(0x0016, 1);

  pump_until_sent();
  sleep_ms(15);

  uart_.inject_rx(make_response(0x01, 0x04, {0x00, 0x04, 0xD2, 0x00}));
  modbus_.loop();

  EXPECT_EQ(device_.received_data.size(), 1);
}

}  // namespace esphome::modbus::testing
