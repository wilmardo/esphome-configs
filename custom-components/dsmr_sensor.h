#include "esphome.h"
using namespace esphome;

// * Max TELEGRAM length
#define P1_MAXLINELENGTH 128

class DsmrSensor : public PollingComponent
{
public:
  sensor::Sensor *consumption_low_tarif = new sensor::Sensor();
  sensor::Sensor *consumption_high_tarif = new sensor::Sensor();
  sensor::Sensor *production_low_tarif = new sensor::Sensor();
  sensor::Sensor *production_high_tarif = new sensor::Sensor();
  sensor::Sensor *power_delivered = new sensor::Sensor();
  sensor::Sensor *power_received = new sensor::Sensor();
  sensor::Sensor *instant_power_usage = new sensor::Sensor();
  sensor::Sensor *instant_power_current = new sensor::Sensor();
  sensor::Sensor *gas_meter_m3 = new sensor::Sensor();
  sensor::Sensor *actual_tarif_group = new sensor::Sensor();
  sensor::Sensor *short_power_outages = new sensor::Sensor();
  sensor::Sensor *long_power_outages = new sensor::Sensor();
  sensor::Sensor *short_power_drops = new sensor::Sensor();
  sensor::Sensor *short_power_peaks = new sensor::Sensor();

  // * Set to store received TELEGRAM
  char TELEGRAM[P1_MAXLINELENGTH];

  // * Set to store the data values read
  float CONSUMPTION_LOW_TARIF;
  float CONSUMPTION_HIGH_TARIF;
  float PRODUCTION_LOW_TARIF;
  float PRODUCTION_HIGH_TARIF;
  float POWER_DELIVERED;
  float POWER_RECEIVED;
  byte INSTANT_POWER_CURRENT;
  float INSTANT_POWER_USAGE;
  float GAS_METER_M3;

  // * Set to store data counters read
  byte ACTUAL_TARIF;
  byte SHORT_POWER_OUTAGES;
  byte LONG_POWER_OUTAGES;
  byte SHORT_POWER_DROPS;
  byte SHORT_POWER_PEAKS;

  // * Set during CRC checking
  bool VALID_CRC_FOUND = false;
  unsigned int currentCRC = 0;

  DsmrSensor() : PollingComponent(5000) {}

  void setup() override
  {
    Serial.begin(115200);
    pinMode(14, OUTPUT);
  }

  void loop() override
  {
    if (Serial.available())
    {
      memset(TELEGRAM, 0, sizeof(TELEGRAM));

      while (Serial.available())
      {
        int len = Serial.readBytesUntil('\n', TELEGRAM, P1_MAXLINELENGTH);
        TELEGRAM[len] = '\n';
        TELEGRAM[len + 1] = 0;
        yield();

        decodeTelegram(len + 1);
      }
    }
  }

  void update() override
  {
    if (VALID_CRC_FOUND)
    {
      consumption_low_tarif->publish_state(CONSUMPTION_LOW_TARIF);
      consumption_high_tarif->publish_state(CONSUMPTION_HIGH_TARIF);
      production_low_tarif->publish_state(PRODUCTION_LOW_TARIF);
      production_high_tarif->publish_state(PRODUCTION_HIGH_TARIF);
      power_delivered->publish_state(POWER_DELIVERED);
      power_received->publish_state(POWER_RECEIVED);
      instant_power_usage->publish_state(INSTANT_POWER_USAGE);
      instant_power_current->publish_state(INSTANT_POWER_CURRENT);
      gas_meter_m3->publish_state(GAS_METER_M3);

      actual_tarif_group->publish_state(ACTUAL_TARIF);
      short_power_outages->publish_state(SHORT_POWER_OUTAGES);
      long_power_outages->publish_state(LONG_POWER_OUTAGES);
      short_power_drops->publish_state(SHORT_POWER_DROPS);
      short_power_peaks->publish_state(SHORT_POWER_PEAKS);
    }
  }

  unsigned int CRC16(unsigned int crc, unsigned char *buf, int len)
  {
    for (int pos = 0; pos < len; pos++)
    {
      crc ^= (unsigned int)buf[pos]; // * XOR byte into least sig. byte of crc
                                     // * Loop over each bit
      for (int i = 8; i != 0; i--)
      {
        // * If the LSB is set
        if ((crc & 0x0001) != 0)
        {
          // * Shift right and XOR 0xA001
          crc >>= 1;
          crc ^= 0xA001;
        }
        // * Else LSB is not set
        else
          // * Just shift right
          crc >>= 1;
      }
    }
    return crc;
  }

  bool isNumber(char *res, int len)
  {
    for (int i = 0; i < len; i++)
    {
      if (((res[i] < '0') || (res[i] > '9')) && (res[i] != '.' && res[i] != 0))
        return false;
    }
    return true;
  }

  int findCharInArrayRev(char array[], char c, int len)
  {
    for (int i = len - 1; i >= 0; i--)
    {
      if (array[i] == c)
        return i;
    }
    return -1;
  }

  float getValue(char *buffer, int maxlen, char startchar, char endchar)
  {
    int s = findCharInArrayRev(buffer, startchar, maxlen - 2);
    int l = findCharInArrayRev(buffer, endchar, maxlen - 2) - s - 1;

    char res[16];
    memset(res, 0, sizeof(res));

    if (strncpy(res, buffer + s + 1, l))
    {
      if (isNumber(res, l)) {
        return atof(res);
      }
    }
    return 0;
  }

  void decodeTelegram(int len)
  {
    int startChar = findCharInArrayRev(TELEGRAM, '/', len);
    int endChar = findCharInArrayRev(TELEGRAM, '!', len);

    ESP_LOGV("dsmr_mqtt", "TELEGRAM received: %s", TELEGRAM);

    if (startChar >= 0)
    {
      // * Start found. Reset CRC calculation
      currentCRC = CRC16(0x0000, (unsigned char *)TELEGRAM + startChar, len - startChar);
    }
    else if (endChar >= 0)
    {
      // * Add to crc calc
      currentCRC = CRC16(currentCRC, (unsigned char *)TELEGRAM + endChar, 1);

      char messageCRC[5];
      strncpy(messageCRC, TELEGRAM + endChar + 1, 4);

      messageCRC[4] = 0; // * Thanks to HarmOtten (issue 5)
      VALID_CRC_FOUND = (strtol(messageCRC, NULL, 16) == currentCRC);

      if (VALID_CRC_FOUND)
        ESP_LOGI("dsmr_mqtt", "CRC Valid!");
      else
        ESP_LOGE("dsmr_mqtt", "CRC Invalid!");

      currentCRC = 0;
    }
    else
    {
      currentCRC = CRC16(currentCRC, (unsigned char *)TELEGRAM, len);
    }

    // For the spec see p26: https://www.netbeheernederland.nl/_upload/Files/Slimme_meter_15_a727fce1f1.pdf

    // 1-0:1.8.1(000992.992*kWh)
    // 1-0:1.8.1 = Meter Reading electricity deliveredto client (Tariff1) in 0,001 kWh
    if (strncmp(TELEGRAM, "1-0:1.8.1", strlen("1-0:1.8.1")) == 0)
    {
      CONSUMPTION_LOW_TARIF = getValue(TELEGRAM, len, '(', '*');
    }

    // 1-0:1.8.2(000560.157*kWh)
    // 1-0:1.8.2 = Meter Reading electricity deliveredto client (Tariff 2) in 0,001 kWh
    if (strncmp(TELEGRAM, "1-0:1.8.2", strlen("1-0:1.8.2")) == 0)
    {
      CONSUMPTION_HIGH_TARIF = getValue(TELEGRAM, len, '(', '*');
    }

    // 1-0:2.8.1(000348.890*kWh)
    // 1-0:2.8.1 = Meter Reading electricity deliveredby client (Tariff1) in 0,001 kWh
    if (strncmp(TELEGRAM, "1-0:2.8.1", strlen("1-0:2.8.1")) == 0)
    {
      PRODUCTION_LOW_TARIF = getValue(TELEGRAM, len, '(', '*');
    }

    // 1-0:2.8.2(000859.885*kWh)
    // 1-0:2.8.2 = Meter Reading electricity deliveredby client (Tariff2) in 0,001 kWh
    if (strncmp(TELEGRAM, "1-0:2.8.2", strlen("1-0:2.8.2")) == 0)
    {
      PRODUCTION_HIGH_TARIF = getValue(TELEGRAM, len, '(', '*');
    }

    // 0-0:96.14.0(0001)
    // 0-0:96.14.0 = Tariff indicator electricity.
    if (strncmp(TELEGRAM, "0-0:96.14.0", strlen("0-0:96.14.0")) == 0)
    {
      ACTUAL_TARIF = getValue(TELEGRAM, len, '(', ')');
    }

    // 1-0:1.7.0(00.424*kW)
    // 1-0:1.7.0 = Actual electricity powerdelivered (+P)in 1 Wattreso-lution
    if (strncmp(TELEGRAM, "1-0:1.7.0", strlen("1-0:1.7.0")) == 0)
    {
      POWER_DELIVERED = getValue(TELEGRAM, len, '(', '*');
    }

    // 1-0:2.7.0(00.000*kW)
    // Actual electricity power received (-P) in 1 Wattresolution
    if (strncmp(TELEGRAM, "1-0:2.7.0", strlen("1-0:2.7.0")) == 0)
    {
      POWER_RECEIVED = getValue(TELEGRAM, len, '(', '*');
    }

    // 0-0:96.7.21(00003)
    // 0-0:96.7.21 = Number of power failures in any phase
    if (strncmp(TELEGRAM, "0-0:96.7.21", strlen("0-0:96.7.21")) == 0)
    {
      SHORT_POWER_OUTAGES = getValue(TELEGRAM, len, '(', ')');
    }

    // 0-0:96.7.9(00001)
    // 0-0:96.7.9 = Number of long power failures in any phase
    if (strncmp(TELEGRAM, "0-0:96.7.9", strlen("0-0:96.7.9")) == 0)
    {
      LONG_POWER_OUTAGES = getValue(TELEGRAM, len, '(', ')');
    }

    // 1-0:32.32.0(00000)
    // 1-0:32.32.0 = Number of voltage sags in phase L1
    if (strncmp(TELEGRAM, "1-0:32.32.0", strlen("1-0:32.32.0")) == 0)
    {
      SHORT_POWER_DROPS = getValue(TELEGRAM, len, '(', ')');
    }

    // 1-0:32.36.0(00000)
    // 1-0:32.36.0 = Number of voltage swells in phase L1
    if (strncmp(TELEGRAM, "1-0:32.36.0", strlen("1-0:32.36.0")) == 0)
    {
      SHORT_POWER_PEAKS = getValue(TELEGRAM, len, '(', ')');
    }

    // 1-0:31.7.0(002*A)
    // 1-0:31.7.0 = Instantaneous current L1 in A resolu-tion.
    if (strncmp(TELEGRAM, "1-0:31.7.0", strlen("1-0:31.7.0")) == 0)
    {
      INSTANT_POWER_CURRENT = getValue(TELEGRAM, len, '(', '*');
    }

    // 1-0:21.7.0(00.378*kW)
    // 1-0:21.7.0 = Instantaneous active power L1 (+P) in W resolution
    if (strncmp(TELEGRAM, "1-0:21.7.0", strlen("1-0:21.7.0")) == 0)
    {
      INSTANT_POWER_USAGE = getValue(TELEGRAM, len, '(', '*');
    }

    // 0-1:24.2.1(190811180000S)(01916.969*m3)
    // 0-1:24.2.1 = Last 5-minute Meter reading in 0,001 m3 and capture time
    if (strncmp(TELEGRAM, "0-1:24.2.1", strlen("0-1:24.2.1")) == 0)
    {
      GAS_METER_M3 = getValue(TELEGRAM, len, '(', '*');
    }
  }
};
