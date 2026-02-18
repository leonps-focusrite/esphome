#include "daikin_madoka.h"

#include "esphome/core/log.h"
#include <utility>

#ifdef USE_ESP32

namespace esphome::daikin_madoka {

using namespace esphome::climate;

static const uint16_t CMD_GET_SETTING_STATUS = 0x0020;
static const uint16_t CMD_SET_SETTING_STATUS = 0x4020;
static const uint16_t CMD_GET_OPERATION_MODE = 0x0030;
static const uint16_t CMD_SET_OPERATION_MODE = 0x4030;
static const uint16_t CMD_GET_SETPOINT = 0x0040;
static const uint16_t CMD_SET_SETPOINT = 0x4040;
static const uint16_t CMD_GET_FAN_SPEED = 0x0050;
static const uint16_t CMD_SET_FAN_SPEED = 0x4050;
static const uint16_t CMD_GET_SENSOR_INFORMATION = 0x0110;

void DaikinMadoka::dump_config() { LOG_CLIMATE(TAG, "Daikin Madoka Climate Controller", this); }

void DaikinMadoka::setup() { this->receive_semaphore_ = xSemaphoreCreateMutex(); }

inline static uint32_t get_command_cooldown(uint16_t cmd) {
  switch (cmd) {
    case CMD_GET_SETTING_STATUS:
    case CMD_GET_OPERATION_MODE:
    case CMD_GET_SETPOINT:
    case CMD_GET_FAN_SPEED:
    case CMD_GET_SENSOR_INFORMATION:
      return 50;
    case CMD_SET_SETTING_STATUS:
      return 200;
    case CMD_SET_OPERATION_MODE:
      return 600;
    case CMD_SET_SETPOINT:
      return 400;
    case CMD_SET_FAN_SPEED:
      return 200;
    default:
      return 100;
  }
}

void DaikinMadoka::loop() {
  ESP_LOGV(TAG, "loop() called");
  std::vector<uint8_t> chk = {};
  if (xSemaphoreTake(this->receive_semaphore_, 0L)) {
    ESP_LOGVV(TAG, "Semaphore taken for received_chunks_");
    if (!this->received_chunks_.empty()) {
      ESP_LOGVV(TAG, "Received chunk available, popping");
      chk = this->received_chunks_.front();
      this->received_chunks_.pop();
    }
    xSemaphoreGive(this->receive_semaphore_);
    if (!chk.empty()) {
      ESP_LOGVV(TAG, "Processing incoming chunk of size %zu", chk.size());
      this->process_incoming_chunk_(chk);
    }
  }
  if (!this->query_queue_.empty() && !this->pending_message_) {
    Query query = this->query_queue_.front();
    this->query_queue_.pop();
    this->query_(query.cmd, query.args);
    this->pending_message_ = true;
    this->set_timeout("query", get_command_cooldown(query.cmd), [this]() { this->pending_message_ = false; });
  }
  if (this->should_update_) {
    ESP_LOGD(TAG, "should_update_ is true, calling update()");
    this->should_update_ = false;
    this->update();
  }
}

void DaikinMadoka::control(const ClimateCall &call) {
  if (this->node_state != espbt::ClientState::ESTABLISHED)
    return;

  auto mode_opt = call.get_mode();
  if (mode_opt.has_value()) {
    ClimateMode mode = mode_opt.value();
    uint8_t mode_out = 255, status_out = 0;
    switch (mode) {
      case climate::CLIMATE_MODE_OFF:
        status_out = 0;
        break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        status_out = 1;
        mode_out = 2;
        break;
      case climate::CLIMATE_MODE_COOL:
        status_out = 1;
        mode_out = 3;
        break;
      case climate::CLIMATE_MODE_HEAT:
        status_out = 1;
        mode_out = 4;
        break;
      case climate::CLIMATE_MODE_FAN_ONLY:
        status_out = 1;
        mode_out = 0;
        break;
      case climate::CLIMATE_MODE_DRY:
        status_out = 1;
        mode_out = 1;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported mode: %d", mode);
        break;
    }
    ESP_LOGD(TAG, "status: %d, mode: %d", status_out, mode_out);
    if (mode_out != 255) {
      this->query_queue_.push({CMD_SET_OPERATION_MODE, std::vector<uint8_t>{0x20, 0x01, (uint8_t) mode_out}});
    }
    this->query_queue_.push({CMD_SET_SETTING_STATUS, std::vector<uint8_t>{0x20, 0x01, (uint8_t) status_out}});
  }
  std::vector<uint8_t> temp_setpoint_args;
  auto target_temperature_high_opt = call.get_target_temperature_high();
  if (target_temperature_high_opt.has_value()) {
    uint16_t target_high = target_temperature_high_opt.value() * 128;
    temp_setpoint_args.insert(temp_setpoint_args.end(),
                              {0x20, 0x02, (uint8_t) ((target_high >> 8) & 0xFF), (uint8_t) (target_high & 0xFF)});
  }
  auto target_temperature_low_opt = call.get_target_temperature_low();
  if (target_temperature_low_opt.has_value()) {
    uint16_t target_low = target_temperature_low_opt.value() * 128;
    temp_setpoint_args.insert(temp_setpoint_args.end(),
                              {0x21, 0x02, (uint8_t) ((target_low >> 8) & 0xFF), (uint8_t) (target_low & 0xFF)});
  }
  if (!temp_setpoint_args.empty()) {
    this->query_queue_.push({CMD_SET_SETPOINT, temp_setpoint_args});
  }
  auto fan_mode_opt = call.get_fan_mode();
  if (fan_mode_opt.has_value()) {
    uint8_t fan_mode = fan_mode_opt.value();
    uint8_t fan_mode_out = 255;
    switch (fan_mode) {
      case climate::CLIMATE_FAN_AUTO:
        fan_mode_out = 0;
        break;
      case climate::CLIMATE_FAN_LOW:
        fan_mode_out = 1;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        fan_mode_out = 3;
        break;
      case climate::CLIMATE_FAN_HIGH:
        fan_mode_out = 5;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported fan mode: %d", fan_mode);
        break;
    }
    if (fan_mode_out != 255) {
      this->query_queue_.push({CMD_SET_FAN_SPEED, std::vector<uint8_t>{0x20, 0x01, (uint8_t) fan_mode_out, 0x21, 0x01,
                                                                       (uint8_t) fan_mode_out}});
    }
  }
  ESP_LOGD(TAG, "Setting should_update_ to true");
  this->should_update_ = true;
}

void DaikinMadoka::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  ESP_LOGV(TAG, "gap_event_handler() called, event=%d", event);
  switch (event) {
    case ESP_GAP_BLE_SEC_REQ_EVT:
      ESP_LOGD(TAG, "ESP_GAP_BLE_SEC_REQ_EVT");
      esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      break;
    case ESP_GAP_BLE_NC_REQ_EVT:
      ESP_LOGD(TAG, "ESP_GAP_BLE_NC_REQ_EVT");
      esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
      ESP_LOGI(TAG, "ESP_GAP_BLE_NC_REQ_EVT, the passkey Notify number: %06d", param->ble_security.key_notif.passkey);
      break;
    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      ESP_LOGD(TAG, "ESP_GAP_BLE_AUTH_CMPL_EVT");
      if (!param->ble_security.auth_cmpl.success) {
        ESP_LOGE(TAG, "Authentication failed, status: 0x%x", param->ble_security.auth_cmpl.fail_reason);
        break;
      }
      auto *nfy = this->parent_->get_characteristic(MADOKA_SERVICE_UUID, NOTIFY_CHARACTERISTIC_UUID);
      auto *wwr = this->parent_->get_characteristic(MADOKA_SERVICE_UUID, WWR_CHARACTERISTIC_UUID);
      if (nfy == nullptr || wwr == nullptr) {
        ESP_LOGW(TAG, "[%s] No control service found at device, not a Daikin Madoka..?", this->get_name().c_str());
        break;
      }
      this->notify_handle_ = nfy->handle;
      this->wwr_handle_ = wwr->handle;

      ESP_LOGD(TAG, "Registering for notify, handle=%d", nfy->handle);
      auto status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(), this->parent_->get_remote_bda(),
                                                      nfy->handle);
      if (status) {
        ESP_LOGW(TAG, "[%s] esp_ble_gattc_register_for_notify failed, status=%d", this->get_name().c_str(), status);
      }
      break;
    }
    default:
      ESP_LOGV(TAG, "Unhandled GAP event: %d", event);
      break;
  }
}

void DaikinMadoka::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                       esp_ble_gattc_cb_param_t *param) {
  ESP_LOGV(TAG, "gattc_event_handler() called, event=%d", event);
  switch (event) {
    case ESP_GATTC_DISCONNECT_EVT: {
      this->node_state = espbt::ClientState::IDLE;
      this->current_temperature = NAN;
      this->target_temperature = NAN;
      this->publish_state();
      this->pending_chunks_.clear();
      this->should_update_ = false;
      this->query_queue_ = {};
      this->pending_message_ = false;
      break;
    }
    case ESP_GATTC_WRITE_DESCR_EVT:
      ESP_LOGD(TAG, "GATT write descriptor event, status=%d", param->write.status);
      if (param->write.status != ESP_GATT_OK) {
        if (param->write.status == ESP_GATT_INSUF_AUTHENTICATION) {
          ESP_LOGE(TAG, "Insufficient authentication");
        } else {
          ESP_LOGE(TAG, "Failed writing characteristic descriptor, status = 0x%x", param->write.status);
        }
      }
      break;
    case ESP_GATTC_SEARCH_CMPL_EVT: {
      ESP_LOGD(TAG, "GATT search complete, setting encryption");
      esp_ble_set_encryption(this->parent_->get_remote_bda(), ESP_BLE_SEC_ENCRYPT_MITM);
      break;
    }
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
      this->node_state = espbt::ClientState::ESTABLISHED;
      this->should_update_ = true;
      break;
    }
    case ESP_GATTC_NOTIFY_EVT: {
      ESP_LOGVV(TAG, "GATT notify event, handle=%d, expected=%d, len=%d", param->notify.handle, this->notify_handle_,
                param->notify.value_len);
      if (param->notify.handle != this->notify_handle_) {
        ESP_LOGW(TAG, "Different notify handle");
        break;
      }
      std::vector<uint8_t> chk =
          std::vector<uint8_t>{param->notify.value, param->notify.value + param->notify.value_len};
      xSemaphoreTake(this->receive_semaphore_, portMAX_DELAY);
      this->received_chunks_.push(chk);
      xSemaphoreGive(this->receive_semaphore_);
      ESP_LOGVV(TAG, "Pushed received chunk of size %zu", chk.size());
      break;
    }
    default:
      ESP_LOGV(TAG, "Unhandled GATTC event: %d", event);
      break;
  }
}

void DaikinMadoka::update() {
  ESP_LOGD(TAG, "update() called. Got update request...");
  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGD(TAG, "...but device is disconnected");
    return;
  }

  std::vector<uint16_t> all_cmds{CMD_GET_SETTING_STATUS, CMD_GET_OPERATION_MODE, CMD_GET_SETPOINT, CMD_GET_FAN_SPEED,
                                 CMD_GET_SENSOR_INFORMATION};
  ESP_LOGD(TAG, "Sending %zu update queries", all_cmds.size());
  for (auto cmd : all_cmds) {
    this->query_queue_.push({cmd, std::vector<uint8_t>{0x00, 0x00}});
  }
}

static bool validate_buffer(std::vector<uint8_t> buffer) { return !buffer.empty() && buffer[0] == buffer.size(); }

void DaikinMadoka::process_incoming_chunk_(std::vector<uint8_t> chk) {
  ESP_LOGVV(TAG, "process_incoming_chunk_ called, chunk size=%zu", chk.size());
  if (chk.size() < 2) {
    ESP_LOGI(TAG, "Chunk discarded: invalid length (%zu).", chk.size());
    return;
  }
  uint8_t chunk_id = chk[0];
  ESP_LOGVV(TAG, "Chunk ID: %d", chunk_id);
  std::vector<uint8_t> stripped{chk.begin() + 1, chk.end()};
  if (chunk_id == 0 && validate_buffer(stripped)) {
    ESP_LOGVV(TAG, "Single chunk, valid buffer. Parsing.");
    this->parse_cb_(stripped);
    return;
  }
  if (chunk_id == 0 && !this->pending_chunks_.empty()) {
    ESP_LOGW(TAG, "Buffer is not empty, but new message started. Clearing buffer.");
    this->pending_chunks_.clear();
  }
  if (this->pending_chunks_.contains(chunk_id)) {
    ESP_LOGE(TAG, "Another packet with the same chunk ID is already in the buffer.");
    ESP_LOGD(TAG, "Chunk ID: %d.", chunk_id);
    return;
  }
  this->pending_chunks_[chunk_id] = chk;

  ESP_LOGVV(TAG, "pending_chunks_ size: %zu, highest chunk ID: %d", this->pending_chunks_.size(),
            this->pending_chunks_.rbegin()->first);
  if (this->pending_chunks_.size() != this->pending_chunks_.rbegin()->first + 1) {
    ESP_LOGW(TAG, "Buffer is missing packets");
    return;
  }

  std::vector<uint8_t> msg;
  int lim = this->pending_chunks_.size();
  for (int i = 0; i < lim; i++) {
    msg.insert(msg.end(), this->pending_chunks_[i].begin() + 1, this->pending_chunks_[i].end());
  }
  ESP_LOGVV(TAG, "Reassembled message size: %zu", msg.size());
  if (validate_buffer(msg)) {
    ESP_LOGVV(TAG, "Buffer valid, clearing pending_chunks_ and parsing");
    this->pending_chunks_.clear();
    this->parse_cb_(msg);
  }
}

std::vector<std::vector<uint8_t>> DaikinMadoka::split_payload_(std::vector<uint8_t> msg) {
  ESP_LOGVV(TAG, "split_payload_ called, msg size=%zu", msg.size());
  std::vector<std::vector<uint8_t>> result;
  size_t len = msg.size();

  // Add leading length byte
  std::vector<uint8_t> buf{(uint8_t) (len + 1)};
  buf.insert(buf.end(), msg.begin(), msg.end());

  for (size_t i = 0; i <= len / (MAX_CHUNK_SIZE - 1); i++) {
    std::vector<uint8_t> chunk{(uint8_t) i};
    chunk.insert(chunk.end(), buf.begin() + (i * (MAX_CHUNK_SIZE - 1)),
                 std::min(buf.end(), buf.begin() + ((i + 1) * (MAX_CHUNK_SIZE - 1))));

    ESP_LOGVV(TAG, "Created chunk %zu, size=%zu", i, chunk.size());
    result.push_back(chunk);
  }

  return result;
}

std::vector<uint8_t> DaikinMadoka::prepare_message_(uint16_t cmd, std::vector<uint8_t> args) {
  ESP_LOGVV(TAG, "prepare_message_ called, cmd=0x%04X, args size=%zu", cmd, args.size());
  std::vector<uint8_t> result({0x00, (uint8_t) ((cmd >> 8) & 0xFF), (uint8_t) (cmd & 0xFF)});
  result.insert(result.end(), args.begin(), args.end());
  ESP_LOGVV(TAG, "Prepared message size: %zu", result.size());
  return result;
}

void DaikinMadoka::query_(uint16_t cmd, std::vector<uint8_t> args) {
  std::vector<uint8_t> payload = this->prepare_message_(cmd, std::move(args));

  if (this->node_state != espbt::ClientState::ESTABLISHED) {
    ESP_LOGW(TAG, "query_ called but node_state is not ESTABLISHED");
    return;
  }
  const auto chunks = this->split_payload_(payload);
  const char *addr = this->parent_->address_str();
  for (const auto &chk : chunks) {
    esp_err_t status = ESP_OK;
    for (int j = 0; j < BLE_SEND_MAX_RETRIES; j++) {
      status = esp_ble_gattc_write_char(this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->wwr_handle_,
                                        chk.size(), (uint8_t *) chk.data(), ESP_GATT_WRITE_TYPE_NO_RSP,
                                        ESP_GATT_AUTH_REQ_NONE);
      if (!status) {
        ESP_LOGVV(TAG, "Write char success on try %d", j + 1);
        break;
      }
      ESP_LOGD(TAG, "[%s] esp_ble_gattc_write_char failed (%d of %d), status=%d", addr, j + 1, BLE_SEND_MAX_RETRIES,
               status);
    }
    if (status) {
      ESP_LOGE(TAG, "[%s] Command could not be sent, last status=%d", addr, status);
      return;
    }
  }
}

void DaikinMadoka::parse_cb_(std::vector<uint8_t> msg) {
  if (msg.size() < 4) {
    ESP_LOGE(TAG, "Discarding message: invalid length.");
    return;
  }
  const uint16_t function_id = msg[2] << 8 | msg[3];
  size_t i = 4;
  const size_t message_size = msg.size();

  auto safe_next = [&](uint8_t &out) -> bool {
    if (i >= message_size)
      return false;
    out = msg[i++];
    return true;
  };
  auto safe_next_len = [&](uint8_t &out) -> bool {
    if (i >= message_size)
      return false;
    out = msg[i++];
    return true;
  };
  auto safe_val = [&](uint8_t len) -> std::vector<uint8_t> {
    if (i + len > message_size)
      return {};
    std::vector<uint8_t> val(msg.begin() + i, msg.begin() + i + len);
    i += len;
    return val;
  };

  switch (function_id) {
    case CMD_GET_SETTING_STATUS:
      ESP_LOGVV(TAG, "Parsing CMD_GET_SETTING_STATUS");
      while (i < message_size) {
        uint8_t argument_id, len;
        if (!safe_next(argument_id) || !safe_next_len(len))
          break;
        ESP_LOGVV(TAG, "Arg id: 0x%02X, len: %d", argument_id, len);
        if (argument_id == 0x20) {
          auto val = safe_val(len);
          if (val.empty()) {
            ESP_LOGW(TAG, "CMD_GET_SETTING_STATUS: value out of bounds");
            break;
          }
          this->cur_status_.status = val[0];
          ESP_LOGD(TAG, "cur_status_.status = %d", this->cur_status_.status);
        } else {
          i += len;
        }
      }
      break;
    case CMD_GET_OPERATION_MODE:
      ESP_LOGVV(TAG, "Parsing CMD_GET_OPERATION_MODE");
      while (i < message_size) {
        uint8_t argument_id, len;
        if (!safe_next(argument_id) || !safe_next_len(len))
          break;
        ESP_LOGVV(TAG, "Arg id: 0x%02X, len: %d", argument_id, len);
        if (argument_id == 0x20) {
          auto val = safe_val(len);
          if (val.empty()) {
            ESP_LOGW(TAG, "CMD_GET_OPERATION_MODE: value out of bounds");
            break;
          }
          this->cur_status_.mode = val[0];
          ESP_LOGD(TAG, "cur_status_.mode = %d", this->cur_status_.mode);
        } else {
          i += len;
        }
      }
      break;
    default:
      ESP_LOGVV(TAG, "parse_cb_: unhandled function_id 0x%04X", function_id);
      break;
  }
  switch (function_id) {
    case CMD_GET_SETTING_STATUS:
    case CMD_GET_OPERATION_MODE:
      ESP_LOGD(TAG, "status: %d, mode: %d", this->cur_status_.status, this->cur_status_.mode);
      if (this->cur_status_.status) {
        switch (this->cur_status_.mode) {
          case 0:
            this->mode = climate::CLIMATE_MODE_FAN_ONLY;
            ESP_LOGD(TAG, "CLIMATE_MODE_FAN_ONLY");
            break;
          case 1:
            this->mode = climate::CLIMATE_MODE_DRY;
            ESP_LOGD(TAG, "CLIMATE_MODE_DRY");
            break;
          case 2:
            this->mode = climate::CLIMATE_MODE_HEAT_COOL;
            ESP_LOGD(TAG, "CLIMATE_MODE_HEAT_COOL");
            break;
          case 3:
            this->mode = climate::CLIMATE_MODE_COOL;
            ESP_LOGD(TAG, "CLIMATE_MODE_COOL");
            break;
          case 4:
            this->mode = climate::CLIMATE_MODE_HEAT;
            ESP_LOGD(TAG, "CLIMATE_MODE_HEAT");
            break;
        }
      } else {
        this->mode = climate::CLIMATE_MODE_OFF;
        ESP_LOGD(TAG, "CLIMATE_MODE_OFF");
      }
      break;
    case CMD_GET_SETPOINT:
      ESP_LOGVV(TAG, "Parsing CMD_GET_SETPOINT");
      while (i < message_size) {
        uint8_t argument_id, len;
        if (!safe_next(argument_id) || !safe_next_len(len))
          break;
        ESP_LOGVV(TAG, "Arg id: 0x%02X, len: %d", argument_id, len);
        switch (argument_id) {
          case 0x20: {
            auto val = safe_val(len);
            if (val.size() < 2) {
              ESP_LOGW(TAG, "CMD_GET_SETPOINT: value out of bounds");
              break;
            }
            this->target_temperature_high = (float) (val[0] << 8 | val[1]) / 128;
            ESP_LOGD(TAG, "target_temperature_high = %.2f", this->target_temperature_high);
            break;
          }
          case 0x21: {
            auto val = safe_val(len);
            if (val.size() < 2) {
              ESP_LOGW(TAG, "CMD_GET_SETPOINT: value out of bounds");
              break;
            }
            this->target_temperature_low = (float) (val[0] << 8 | val[1]) / 128;
            ESP_LOGD(TAG, "target_temperature_low = %.2f", this->target_temperature_low);
            break;
          }
          default:
            i += len;
            break;
        }
      }
      break;
    case CMD_GET_FAN_SPEED: {
      ESP_LOGVV(TAG, "Parsing CMD_GET_FAN_SPEED");
      uint8_t fan_mode = 255;
      while (i < message_size) {
        uint8_t argument_id, len;
        if (!safe_next(argument_id) || !safe_next_len(len))
          break;
        ESP_LOGVV(TAG, "Arg id: 0x%02X, len: %d", argument_id, len);
        if (this->cur_status_.mode == 1) {
          i += len;
        } else if ((argument_id == 0x21 && len == 1 && this->cur_status_.mode == 4) ||
                   (argument_id == 0x20 && len == 1 && this->cur_status_.mode != 4)) {
          auto val = safe_val(len);
          if (val.empty()) {
            ESP_LOGW(TAG, "CMD_GET_FAN_SPEED: value out of bounds");
            break;
          }
          fan_mode = val[0];
          ESP_LOGD(TAG, "fan_mode = %d", fan_mode);
        } else {
          i += len;
        }
      }
      switch (fan_mode) {
        case 0:
          this->fan_mode = climate::CLIMATE_FAN_AUTO;
          ESP_LOGD(TAG, "CLIMATE_FAN_AUTO");
          break;
        case 1:
          this->fan_mode = climate::CLIMATE_FAN_LOW;
          ESP_LOGD(TAG, "CLIMATE_FAN_LOW");
          break;
        case 2:
        case 3:
        case 4:
          this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
          ESP_LOGD(TAG, "CLIMATE_FAN_MEDIUM");
          break;
        case 5:
          this->fan_mode = climate::CLIMATE_FAN_HIGH;
          ESP_LOGD(TAG, "CLIMATE_FAN_HIGH");
          break;
        default:
          ESP_LOGD(TAG, "Unknown fan_mode: %d", fan_mode);
          break;
      }
      break;
    }
    case CMD_GET_SENSOR_INFORMATION:
      ESP_LOGVV(TAG, "Parsing CMD_GET_SENSOR_INFORMATION");
      while (i < message_size) {
        uint8_t argument_id, len;
        if (!safe_next(argument_id) || !safe_next_len(len))
          break;
        ESP_LOGVV(TAG, "Arg id: 0x%02X, len: %d", argument_id, len);
        if (argument_id == 0x40) {
          auto val = safe_val(len);
          if (val.empty()) {
            ESP_LOGW(TAG, "CMD_GET_SENSOR_INFORMATION: value out of bounds");
            break;
          }
          this->current_temperature = val[0];
          ESP_LOGD(TAG, "current_temperature = %d", this->current_temperature);
        } else {
          i += len;
        }
      }
      break;
    default:
      ESP_LOGVV(TAG, "parse_cb_: unhandled function_id 0x%04X (2nd switch)", function_id);
      break;
  }

  ESP_LOGD(TAG, "Publishing state");
  this->publish_state();
}

}  // namespace esphome::daikin_madoka

#endif
