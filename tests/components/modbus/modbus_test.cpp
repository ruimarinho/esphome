#include <gtest/gtest.h>

#include "esphome/components/modbus/modbus.h"
#include "esphome/core/helpers.h"

namespace esphome::modbus {

class TestModbus : public Modbus {
 public:
  void append_rx_bytes(const std::vector<uint8_t> &bytes) {
    this->rx_buffer_.insert(this->rx_buffer_.end(), bytes.begin(), bytes.end());
  }
  void run_drop_impossible() { this->drop_impossible_leading_bytes_(); }
  void run_extract() { this->try_extract_frame_(); }
  void set_waiting(uint8_t addr) { this->waiting_for_response_ = addr; }
  size_t rx_buffer_size() const { return this->rx_buffer_.size(); }
  uint32_t parse_failures() const { return this->parse_failure_count_; }
  uint32_t resync_recoveries() const { return this->resync_recovery_count_; }
  uint32_t impossible_leading_drops() const { return this->impossible_leading_byte_drop_count_; }
};

class MockDevice : public ModbusDevice {
 public:
  void on_modbus_data(const std::vector<uint8_t> &data) override { this->payloads.push_back(data); }

  std::vector<std::vector<uint8_t>> payloads;
};

static std::vector<uint8_t> make_rtu_frame(const std::vector<uint8_t> &frame_data) {
  std::vector<uint8_t> frame = frame_data;
  uint16_t crc = esphome::crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back((crc >> 8) & 0xFF);
  return frame;
}

TEST(ModbusTest, TruncatedHeaderRemainsBuffered) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  modbus.append_rx_bytes({0x01, 0x03});
  modbus.run_extract();

  EXPECT_EQ(modbus.rx_buffer_size(), 2);
  EXPECT_EQ(modbus.parse_failures(), 0);
  EXPECT_EQ(modbus.resync_recoveries(), 0);
}

TEST(ModbusTest, StandardFrameDispatchesFromBufferedParser) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  MockDevice device;
  device.set_parent(&modbus);
  device.set_address(0x01);
  modbus.register_device(&device);
  modbus.set_waiting(0x01);

  modbus.append_rx_bytes(make_rtu_frame({0x01, 0x03, 0x02, 0x12, 0x34}));
  modbus.run_extract();

  ASSERT_EQ(device.payloads.size(), 1);
  EXPECT_EQ(device.payloads[0], (std::vector<uint8_t>{0x12, 0x34}));
  EXPECT_EQ(modbus.rx_buffer_size(), 0);
}

TEST(ModbusTest, UserDefinedFourByteFrameDispatches) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  MockDevice device;
  device.set_parent(&modbus);
  device.set_address(0x01);
  modbus.register_device(&device);
  modbus.set_waiting(0x01);

  modbus.append_rx_bytes(make_rtu_frame({0x01, 0x44}));
  modbus.run_extract();

  ASSERT_EQ(device.payloads.size(), 1);
  EXPECT_EQ(device.payloads[0], (std::vector<uint8_t>{0x44}));
  EXPECT_EQ(modbus.rx_buffer_size(), 0);
}

TEST(ModbusTest, SlidingResyncRecoversValidFrameAfterNoise) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  MockDevice device;
  device.set_parent(&modbus);
  device.set_address(0x01);
  modbus.register_device(&device);
  modbus.set_waiting(0x01);

  std::vector<uint8_t> frame = make_rtu_frame({0x01, 0x03, 0x02, 0x12, 0x34});
  std::vector<uint8_t> noisy_frame = {0xAA, 0xBB};
  noisy_frame.insert(noisy_frame.end(), frame.begin(), frame.end());

  modbus.append_rx_bytes(noisy_frame);
  modbus.run_extract();

  ASSERT_EQ(device.payloads.size(), 1);
  EXPECT_EQ(device.payloads[0], (std::vector<uint8_t>{0x12, 0x34}));
  EXPECT_EQ(modbus.parse_failures(), 1);
  EXPECT_EQ(modbus.resync_recoveries(), 1);
  EXPECT_EQ(modbus.rx_buffer_size(), 0);
}

TEST(ModbusTest, DropsImpossibleLeadingByteBeforeResyncWhenWaitingForResponse) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  MockDevice device;
  device.set_parent(&modbus);
  device.set_address(0x01);
  modbus.register_device(&device);
  modbus.set_waiting(0x01);

  std::vector<uint8_t> frame = make_rtu_frame({0x01, 0x03, 0x02, 0x12, 0x34});
  std::vector<uint8_t> with_noise = {0x00};
  with_noise.insert(with_noise.end(), frame.begin(), frame.end());

  modbus.append_rx_bytes(with_noise);
  modbus.run_drop_impossible();
  modbus.run_extract();

  ASSERT_EQ(device.payloads.size(), 1);
  EXPECT_EQ(device.payloads[0], (std::vector<uint8_t>{0x12, 0x34}));
  EXPECT_EQ(modbus.impossible_leading_drops(), 1);
  EXPECT_EQ(modbus.parse_failures(), 0);
  EXPECT_EQ(modbus.rx_buffer_size(), 0);
}

TEST(ModbusTest, NonMatchingLeadingByteStillUsesSlidingResyncWhenNotWaiting) {
  TestModbus modbus;
  modbus.set_role(ModbusRole::CLIENT);

  MockDevice device;
  device.set_parent(&modbus);
  device.set_address(0x01);
  modbus.register_device(&device);

  std::vector<uint8_t> frame = make_rtu_frame({0x01, 0x03, 0x02, 0x12, 0x34});
  std::vector<uint8_t> with_noise = {0xAA};
  with_noise.insert(with_noise.end(), frame.begin(), frame.end());

  modbus.append_rx_bytes(with_noise);
  modbus.run_drop_impossible();
  modbus.run_extract();

  ASSERT_EQ(device.payloads.size(), 1);
  EXPECT_EQ(device.payloads[0], (std::vector<uint8_t>{0x12, 0x34}));
  EXPECT_EQ(modbus.impossible_leading_drops(), 0);
  EXPECT_EQ(modbus.parse_failures(), 1);
  EXPECT_EQ(modbus.resync_recoveries(), 1);
  EXPECT_EQ(modbus.rx_buffer_size(), 0);
}

}  // namespace esphome::modbus
