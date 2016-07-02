#include <SdFat.h>
#include <SPI.h>

#include "mk_serial.h"
#include "mavlink.hpp"
#include "tera_ranger.h"
#include "ublox.h"

// Pin definitions
#define GREEN_LED (7)
#define BUTTON (12)
#define LED1 (32)
#define LED2 (38)
#define LED3 (44)
#define LED4 (50)

#define CARD_PRESENT (3)
#define CARD_SELECT (4)

struct GPSTime {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
} gps_time = { 0 };

// SD card
SdFat sd;
File data_file;

// UBlox serial protocol
UBlox ublox_serial(Serial1);

// Mikrokopter serial protocol
MKSerial mk_serial(Serial2);

// TeraRanger
TeraRanger tera_ranger(Serial2);

// Mikrokopter serial protocol
MKSerial mk_mag(Serial3);

// MAVLink-based sensor
MAVLink px4flow;

bool sd_initialized;

void LogGPSPosLLH(void)
{
  UBXPosLLH * ubx_pos_llh = reinterpret_cast<UBXPosLLH *>(ublox_serial.Data());
  data_file.print("1,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(ubx_pos_llh->gps_ms_time_of_week); data_file.print(',');
  data_file.print(ubx_pos_llh->longitutde); data_file.print(',');
  data_file.print(ubx_pos_llh->latitude); data_file.print(',');
  data_file.print(ubx_pos_llh->height_mean_sea_level); data_file.print(',');
  data_file.print(ubx_pos_llh->horizontal_accuracy); data_file.print(',');
  data_file.println(ubx_pos_llh->vertical_accuracy);
}

void LogGPSVelNED(void)
{
  UBXVelNED * ubx_vel_ned = reinterpret_cast<UBXVelNED *>(ublox_serial.Data());
  data_file.print("2,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(ubx_vel_ned->gps_ms_time_of_week); data_file.print(',');
  data_file.print(ubx_vel_ned->velocity_north); data_file.print(',');
  data_file.print(ubx_vel_ned->velocity_east); data_file.print(',');
  data_file.print(ubx_vel_ned->velocity_down); data_file.print(',');
  data_file.print(ubx_vel_ned->total_speed); data_file.print(',');
  data_file.print(ubx_vel_ned->horizontal_speed); data_file.print(',');
  data_file.print(ubx_vel_ned->course); data_file.print(',');
  data_file.print(ubx_vel_ned->speed_accuracy); data_file.print(',');
  data_file.println(ubx_vel_ned->course_accuracy);
}

void LogGPSSol(void)
{
  UBXSol * ubx_sol = reinterpret_cast<UBXSol *>(ublox_serial.Data());
  data_file.print("3,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(ubx_sol->gps_ms_time_of_week); data_file.print(',');
  data_file.print((int)ubx_sol->gps_fix_type); data_file.print(',');
  data_file.print((int)ubx_sol->gps_fix_status_flags); data_file.print(',');
  data_file.println((int)ubx_sol->number_of_satelites_used);
}

void LogFCSensorData(void)
{
  FCSensorData * fc_sensor_data
    = reinterpret_cast<FCSensorData *>(mk_serial.Data());
  data_file.print("0,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(fc_sensor_data->accelerometer_sum[0]); data_file.print(',');
  data_file.print(fc_sensor_data->accelerometer_sum[1]); data_file.print(',');
  data_file.print(fc_sensor_data->accelerometer_sum[2]); data_file.print(',');
  data_file.print(fc_sensor_data->gyro_sum[0]); data_file.print(',');
  data_file.print(fc_sensor_data->gyro_sum[1]); data_file.print(',');
  data_file.print(fc_sensor_data->gyro_sum[2]); data_file.print(',');
  data_file.print(fc_sensor_data->biased_pressure); data_file.print(',');
  data_file.print(fc_sensor_data->counter_128_hz); data_file.print(',');
  data_file.println(fc_sensor_data->led_on);
}

void LogMagData(void)
{
  int16_t * mag_data = reinterpret_cast<int16_t *>(mk_mag.Data());
  data_file.print("4,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(mag_data[0]); data_file.print(',');
  data_file.print(mag_data[1]); data_file.print(',');
  data_file.println(mag_data[2]);
}

void LogOpticalFlow(void)
{
  OpticalFlow * optical_flow = reinterpret_cast<OpticalFlow *>
    (px4flow.Payload());
  data_file.print("5,");

  data_file.print(millis()); data_file.print(',');
  data_file.print((uint32_t)optical_flow->time_usec); data_file.print(',');
  data_file.print(optical_flow->flow_comp_m_x); data_file.print(',');
  data_file.print(optical_flow->flow_comp_m_y); data_file.print(',');
  data_file.println(optical_flow->ground_distance);
  data_file.print(optical_flow->flow_x); data_file.print(',');
  data_file.print(optical_flow->flow_y); data_file.print(',');
  data_file.print(optical_flow->quality); data_file.print(',');
}

void LogOpticalFlowRad(void)
{
  OpticalFlowRad * optical_flow = reinterpret_cast<OpticalFlowRad *>
    (px4flow.Payload());
  data_file.print("6,");

  data_file.print(millis()); data_file.print(',');
  data_file.print((uint32_t)optical_flow->time_usec); data_file.print(',');
  data_file.print(optical_flow->integration_time_us); data_file.print(',');
  data_file.print(optical_flow->integrated_x); data_file.print(',');
  data_file.print(optical_flow->integrated_y); data_file.print(',');
  data_file.print(optical_flow->integrated_xgyro); data_file.print(',');
  data_file.print(optical_flow->integrated_ygyro); data_file.print(',');
  data_file.print(optical_flow->integrated_zgyro); data_file.print(',');
  data_file.print(optical_flow->time_delta_distance_us); data_file.print(',');
  data_file.print(optical_flow->distance); data_file.print(',');
  data_file.print(optical_flow->temperature); data_file.print(',');
  data_file.println(optical_flow->quality);
}

void LogTeraRanger(void)
{
  data_file.print("7,");

  data_file.print(millis()); data_file.print(',');
  data_file.print(tera_ranger.IR()); data_file.print(',');
  data_file.println(tera_ranger.Sonar());
}

void UpdateTime(void)
{
  UBXTimeUTC * ubx_time = reinterpret_cast<UBXTimeUTC *>(ublox_serial.Data());

  if (ubx_time->valid == 0x07)
  {
    gps_time.sec = ubx_time->sec;
    gps_time.min = ubx_time->min;
    gps_time.hour = ubx_time->hour + 9;  // UTC to Japan
    gps_time.day = ubx_time->day;
    gps_time.month = ubx_time->month;
    gps_time.year = ubx_time->year;
    if (gps_time.hour > 23)
    {
      gps_time.hour -= 24;
      gps_time.day += 1;
      switch (gps_time.month)
      {
        case 1: case 3: case 5: case 7: case 8: case 10: case 12:
          if (gps_time.day > 31) goto NEXT_MONTH;
          break;
        case 4: case 6: case 9: case 11:
          if (gps_time.day > 31) goto NEXT_MONTH;
          break;
        case 2: default:
          if ((gps_time.day > 27) && (gps_time.year % 4) || (gps_time.day > 28))
            goto NEXT_MONTH;
          break;
      }
      return;
      NEXT_MONTH:
      gps_time.month += 1;
      if (gps_time.month > 12)
      {
        gps_time.month = 1;
        gps_time.year += 1;
      }
    }
    Serial.println("Updated time from GPS");
  }
  else
  {
    Serial.println("GPS time not valid");
    gps_time.year = 0;
  }
}

void setup()
{
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);
  pinMode(LED3, OUTPUT);
  pinMode(LED4, OUTPUT);
  pinMode(10, OUTPUT);  // Necessary to satisfy SD library???

  pinMode(CARD_PRESENT, INPUT);
  pinMode(CARD_SELECT, OUTPUT);

  digitalWrite(GREEN_LED, LOW);
  digitalWrite(LED1, LOW);
  digitalWrite(LED2, LOW);
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);

  // Debug output over USB programming port.
  Serial.begin(57600);
  Serial.println("University of Tokyo SD Card Logger");

  // Give us a second.
  delay(1000);

  ublox_serial.Init();  // Serial1 (old style)
  // mk_serial.Init();  // Serial2 (old style)
  tera_ranger.Init();  // Serial2 (old style)
  // mk_mag.Init();  // Serial3 (old style)
  Serial3.begin(115200);  // PX4Flow
}

void loop()
{
  static bool logging_active = false;
  static uint8_t button_samples = 0xFF;

  digitalWrite(LED4, digitalRead(CARD_PRESENT));

  // Wait for 7 consecutive sample of button push (low when pushed).
  button_samples = (button_samples << 1) | (digitalRead(BUTTON) == LOW);
  if (button_samples == 0x7F)
  {
    if (!logging_active && (digitalRead(CARD_PRESENT) == LOW)
      && sd.begin(CARD_SELECT, SPI_FULL_SPEED))
    {
      // Form a filename based on milliseconds since turned on.
      char filename[14];
      if (gps_time.year)
      {
        sprintf(filename, "%02u-%02u-%02u.CSV", gps_time.hour,
          gps_time.min, gps_time.sec);
      }
      else
      {
        sprintf(filename, "%08lu.CSV", millis() / 1000);
      }
      data_file = sd.open(filename, FILE_WRITE);

      if (data_file)
      {
        if (gps_time.year)
        {
          data_file.timestamp(T_CREATE, gps_time.year, gps_time.month,
            gps_time.day, gps_time.hour, gps_time.min, gps_time.sec);
          data_file.timestamp(T_WRITE, gps_time.year, gps_time.month,
            gps_time.day, gps_time.hour, gps_time.min, gps_time.sec);
          data_file.timestamp(T_ACCESS, gps_time.year, gps_time.month,
            gps_time.day, gps_time.hour, gps_time.min, gps_time.sec);
        }

        // Request a data stream from the FlightCtrl
        logging_active = true;
        digitalWrite(GREEN_LED, HIGH);
        Serial2.print("#0i?y\r");  // Request data stream (encoded).
        Serial3.print("#0i?y\r");  // Request data stream (encoded).
      }
    }
    else if (logging_active)
    {
      if (data_file) data_file.close();
      logging_active = false;
      digitalWrite(GREEN_LED, LOW);
    }
  }

  // GPS Logging
  ublox_serial.ProcessIncoming();
  if (ublox_serial.IsAvailable())
  {
    if (logging_active)
    {
      digitalWrite(LED1, HIGH);
      switch (ublox_serial.ID())
      {
        case kIDPosLLH:
          LogGPSPosLLH();
          break;
        case kIDVelNED:
          LogGPSVelNED();
          break;
        case kIDSol:
          LogGPSSol();
          break;
        default:
          break;
      }
      digitalWrite(LED1, LOW);
    }
    else if (ublox_serial.ID() == kIDTimeUTC)
    {
      UpdateTime();
    }

    ublox_serial.Pop();
  }
/*
  // MK FlightCtrl Logging
  mk_serial.ProcessIncoming();

  if (mk_serial.IsAvailable())
  {
    if (logging_active)
    {
      digitalWrite(LED2, HIGH);
      LogFCSensorData();
      digitalWrite(LED2, LOW);
    }
    else
    {
      Serial2.print("#0r@B\r");  // Reset data stream (encoded).
    }
    mk_serial.Pop();
  }
*/
  // TeraRanger
  tera_ranger.ProcessIncoming();
  if (tera_ranger.IsAvailable())
  {
    if (logging_active)
    {
      digitalWrite(LED2, HIGH);
      LogTeraRanger();
      Serial.println(tera_ranger.IR());
      digitalWrite(LED2, LOW);
    }
    tera_ranger.Pop();
  }
/*
  // MK Mag Logging
  mk_mag.ProcessIncoming();

  if (mk_mag.IsAvailable())
  {
    if (logging_active)
    {
      digitalWrite(LED3, HIGH);

      LogMagData();

      digitalWrite(LED3, LOW);
    }
    else
    {
      Serial3.print("#0r@B\r");  // Reset data stream (encoded).
    }
    mk_mag.Pop();
  }
*/
  // PX4FLOW
  // NOTE: this class has been restructured and others should also be
  while (Serial3.available())
  {
    uint8_t byte = Serial3.read();
    px4flow.ProcessIncoming(byte);
  }
  if (px4flow.UnreadData())
  {
    px4flow.Pop();
    if (logging_active)
    {
      digitalWrite(LED3, HIGH);
      switch (px4flow.MessageID())
      {
        case 0:  // heartbeat
          break;
        case 100:  // optical flow
          LogOpticalFlow();
          break;
        case 106:  // optical flow rad
          LogOpticalFlowRad();
          break;
        default:
          break;
      }
      digitalWrite(LED3, LOW);
    }
  }
}
