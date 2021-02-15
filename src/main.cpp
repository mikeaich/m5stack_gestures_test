/**
 * Enable and report the gesture support on the FT6336 chip used in the
 * ESP32-based M5Stack Core 2 module.
 *
 * @ref https://m5stack.com/products/m5stack-core2-esp32-iot-development-kit
 *
 * This code was based on studying the Android 7.0 driver for the FT5x06 chip.
 *
 * @ref https://android.googlesource.com/kernel/msm/+/android-7.0.0_r0.1/drivers/input/touchscreen/ft5x06_ts.c
 *
 * Written by Mike Habicher. Copyright and related rights waived via CC0.
 * @ref https://creativecommons.org/publicdomain/zero/1.0/
 *
 * -- Build --
 * This code was written and tested in PlatformIO.
 * @ref https://docs.platformio.org/en/latest/platforms/espressif32.html
 *
 * -- Known Issues --
 * 1. Gesture detection seems to be unreliable. Some gestures are easier to
 *    trigger than others.
 * 2. Gestures are often reported twice. I haven't had a chance to debug this
 *    yet.
 */

#include <M5Core2.h>

#define TEST_GESTURES 1   // undef this to switch to testing raw touches

TaskHandle_t task_i2c_handle = nullptr;

// Ref. https://gist.github.com/egeltje/db23f8c7b22002a30c68dad9b55c0f4d
void IRAM_ATTR i2c_isr(void* params)
{
  BaseType_t task_switch = pdFALSE;

  xTaskNotifyFromISR(task_i2c_handle, 0x1, eSetBits, &task_switch);
  if (task_switch) {
    portYIELD_FROM_ISR();
  }
}

struct TOUCH_DATA
{
  uint8_t event_flag:2;
  uint8_t reserved_0:2;
  uint8_t x_h:4;
  uint8_t x_l;
  uint8_t id:4;
  uint8_t y_h:4;
  uint8_t y_l;
  uint8_t weight;
  uint8_t area:4;
  uint8_t reserved_1:4;

  uint16_t get_x() const { return x_h << 8 | x_l; }
  uint16_t get_y() const { return y_h << 8 | y_l; }
};

static_assert(sizeof(TOUCH_DATA) == 6, "Unexpected size of TOUCH_DATA");

class PRINT_TOUCH_DATA : public Printable
{
public:
  explicit PRINT_TOUCH_DATA(const TOUCH_DATA& touch_data)
    : m_td(touch_data)
  { }

  virtual size_t printTo(Print& p) const
  {
    size_t n = 0;
    n += p.print("event=");
    switch (m_td.event_flag) {
      case 0:
        n += p.print("down");
        break;
      case 1:
        n += p.print("up");
        break;
      case 2:
        n += p.print("contact");
        break;
      case 3:
        n += p.print("none");
        break;
    }

    n += p.print(" | id=0x");
    n += p.print(m_td.id);

    n += p.print(" | x=");
    n += p.print(m_td.get_x());
    n += p.print(", y=");
    n += p.print(m_td.get_y());
  
    n += p.print(" | weight=");
    n += p.print(m_td.weight);
    n += p.print(" | area=");
    n += p.print(m_td.area);

    return n;
  }

private:
  const TOUCH_DATA& m_td;
};

void decode_touch(const uint8_t* data)
{
  const TOUCH_DATA& td = *reinterpret_cast<const TOUCH_DATA*>(data);
  Serial.println(PRINT_TOUCH_DATA(td));
}

void handle_touch()
{
  static size_t event_num = 0;

  Serial.print("touch ");
  Serial.print(event_num++);
  Serial.print(": ");

  uint8_t data[13];
  M5.Touch.ft6336(0x02, sizeof data, &data[0]);

  Serial.print('[');
  bool space = false;
  for (uint8_t d : data) {
    if (space) {
      Serial.print(' ');
    } else {
      space = true;
    }
    Serial.print(d, HEX);
  }
  Serial.print("] ");

  const uint8_t num_touch_points = data[0] & 0xF;
  switch (num_touch_points) {
    case 2:
      decode_touch(data + 7);
      // fallthough!
    case 1:
      decode_touch(data + 1);
      break;
    default:
      Serial.print("num_touch_points=");
      Serial.println(num_touch_points);
      break;
  }
}

/**
 * @brief Dump gesture data to the serial port, and render the shape on the
 * display.
 */
void handle_gesture()
{
  static size_t event_num = 0;

  Serial.print("gesture ");
  Serial.print(event_num++);
  Serial.print(": ");

  // The starting register of the gesture data.
  constexpr uint8_t REG_GESTURE_OUTPUT = 0xD3;

  // The string of bytes read starting at the above register is structured. The
  // following symbols help to decode it.
  constexpr size_t GESTURE_POINTER_SIZEOF = 4;
  constexpr size_t GESTURE_ID_FLAG_SIZE = 1;
  constexpr size_t GESTURE_POINTER_NUM_FLAG_SIZE = 1;
  constexpr size_t GESTURE_SET_FLAG_SIZE = 6;
  constexpr size_t GESTURE_DATA_HEADER =
    GESTURE_ID_FLAG_SIZE + GESTURE_POINTER_NUM_FLAG_SIZE + GESTURE_SET_FLAG_SIZE;

  // Start by reading the gesture header.
  uint8_t data[GESTURE_DATA_HEADER];
  M5.Touch.ft6336(REG_GESTURE_OUTPUT, sizeof data, &data[0]);
  Serial.print('[');
  bool space = false;
  for (uint8_t d : data) {
    if (space) {
      Serial.print(' ');
    } else {
      space = true;
    }
    Serial.print(d, HEX);
  }
  Serial.print("] \"");

  switch (data[0]) {
    case 0x20:
      Serial.print("left");
      break;
    case 0x21:
      Serial.print("right");
      break;
    case 0x22:
      Serial.print("up");
      break;
    case 0x23:
      Serial.print("down");
      break;
    case 0x24:
      Serial.println("double-click");
      return; // nothing more to do!
    case 0x30:
      Serial.print('O');
      break;
    case 0x31:
      Serial.print('W');
      break;
    case 0x32:
      Serial.print('M');
      break;
    case 0x33:
      Serial.print('E');
      break;
    case 0x41:
      Serial.print('Z');
      break;
    case 0x44:
      Serial.print('L');
      break;
    case 0x46:
      Serial.print('S');
      break;
    case 0x52:
      Serial.print('C');
      break;
    case 0x54:
      Serial.print('V');
      break;
    default:
      // Some other 'shapes' get reported as 0x82 and 0x83, though I haven't
      // been able to figure out what the FT6336 thinks they are.
      Serial.print(data[0]);
      break;
  }
  Serial.print("\" ");

  // Figure out how much data to read: data is returned as an array of 12-bit
  // (x, y) points.
  const uint8_t num_points = data[1];
  const size_t gesture_data_size =
    num_points * GESTURE_POINTER_SIZEOF + GESTURE_DATA_HEADER;
  Serial.print('(');
  Serial.print(gesture_data_size);
  Serial.print(") ");
  uint8_t full_data[gesture_data_size];

  // We need to get a lot of data from the touch controller. Rather than chunk
  // it into the Arduino framework's fixed-size buffer, we can read it
  // directly into our own by calling readTransmission(). But first we have to
  // send to address of the register to start reading from.
  Wire1.beginTransmission(CST_DEVICE_ADDR);
  Wire1.write(REG_GESTURE_OUTPUT);
  Wire1.endTransmission();

  size_t num_bytes = 0; // output: used for sanity checking
  const i2c_err_t rv =
    Wire1.readTransmission(CST_DEVICE_ADDR, &full_data[0], gesture_data_size, true, &num_bytes);
  if (rv != I2C_ERROR_OK) {
    Serial.print("i2c_err_t=");
    Serial.println(rv);
    return;
  }

  if (num_bytes != gesture_data_size) {
    Serial.print("bytes=");
    Serial.print(num_bytes);
  }

  M5.Lcd.clear(TFT_BLACK);

  bool link = false;

  // Use int16_t to stay consistent with M5.Lcd.width()/.height().
  int16_t x0;
  int16_t y0;

  for (size_t i = 0; i < num_points; ++i) {
    auto read12 = [&](const size_t p) -> int16_t /* same here */ {
      return (full_data[p] & 0x0F) << 8 | full_data[p + 1];
    };

    const size_t p = GESTURE_POINTER_SIZEOF * i + GESTURE_DATA_HEADER;

    // By default, the m5stack Core 2 display is rotated 90 degrees clockwise
    // from the touch sensor. We adjust for that here by treating the first
    // value as the Y coordinate, and the second as X.
    const auto y1 = read12(p);
    const auto x1 = read12(p + 2);

    if (link) {
      Serial.print('-');
      // The gesture seems to end with values set to 4095.
      if (x1 != 4095 && y1 != 4095) {
        M5.Lcd.drawLine(x0, y0, x1, y1, TFT_YELLOW);
      }
    } else {
      link = true;
    }

    x0 = x1;
    y0 = y1;

    Serial.print('(');
    Serial.print(x1);
    Serial.print(", ");
    Serial.print(y1);
    Serial.print(')');
  }

  Serial.println();
}

void task_i2c(void* args)
{
  Serial.println("I2C Task starting");

  gpio_config_t gpio_cfg;
  gpio_cfg.pin_bit_mask = GPIO_SEL_39; // CST_INT
  gpio_cfg.mode = GPIO_MODE_INPUT;
  // The m5stack core has external pullups on the button pins.
  gpio_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  gpio_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_cfg.intr_type = GPIO_INTR_NEGEDGE;
  gpio_config(&gpio_cfg);

  gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
  gpio_isr_handler_add(static_cast<gpio_num_t>(CST_INT), i2c_isr, reinterpret_cast<void*>(CST_INT));

  M5.Touch.interval(DEFAULT_INTERVAL);
  Serial.print("update interval=");           // verify
  Serial.println(M5.Touch.interval());

#ifdef TEST_GESTURES
  constexpr uint8_t REG_GESTURE_ENABLE = 0xD0;

  M5.Touch.ft6336(REG_GESTURE_ENABLE, 0x01);  // cargo-culted value
  Serial.print("REG_GESTURE_ENABLE=");        // verify
  Serial.println(M5.Touch.ft6336(0xD0));
#endif

  Serial.println("I2C Task started");

  uint32_t notification;

  while (true) {
    BaseType_t rv = xTaskNotifyWait(0, 0, &notification, portMAX_DELAY);
    // BaseType_t rv = xTaskNotifyWait(0, 0, &notification, pdMS_TO_TICKS(100));

    if (rv == pdFALSE) {
      continue;
    }

#ifdef TEST_GESTURES
    handle_gesture();
#else
    handle_touch();
#endif
  }

  vTaskDelete(nullptr);
}

void setup()
{
  M5.begin(true, false, true, false);
  M5.Axp.SetLed(1);

  Serial.println("setup");
  M5.Axp.SetLcdVoltage(3300);

  M5.Axp.SetBusPowerMode(0);
  M5.Axp.SetCHGCurrent(AXP192::kCHG_190mA);

  Serial.print("core=");
  Serial.println(xPortGetCoreID());

  // In order to access the I2C bus, the I2C handling task has to run on  the
  // same core on which the I2C bus was initialized -- regardless of where
  // the interrupt handler was installed! For the m5stack Arduino framework,
  // this means the core where setup() (and M5.begin()) is called: core 1.
  xTaskCreatePinnedToCore(task_i2c, "I2C", 4096, nullptr, 1, &task_i2c_handle, 1);

  Serial.println("setup complete");
}

void loop()
{
  vTaskDelay(portMAX_DELAY); // endless sleep
}
