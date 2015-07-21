#include <SdFat.h>
#include <SPI.h>

#include "mk_serial.h"
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

// SD card
SdFat sd;
File data_file;

// Mikrokopter serial protocol
// MKSerial mk_serial(Serial2);

// UBlox serial protocol
UBlox ublox_serial(Serial1);

bool sd_initialized;

void LogGPSPosLLH(void)
{
  UBXPosLLH * ubx_pos_llh = reinterpret_cast<UBXPosLLH *>(ublox_serial.Data());
  data_file.print("1,");

  data_file.print(ubx_pos_llh->gps_ms_time_of_week); data_file.print(',');
  data_file.print(ubx_pos_llh->longitutde); data_file.print(',');
  data_file.print(ubx_pos_llh->latitude); data_file.print(',');
  data_file.print(ubx_pos_llh->height_above_ellipsoid); data_file.print(',');
  data_file.print(ubx_pos_llh->height_mean_sea_level); data_file.print(',');
  data_file.print(ubx_pos_llh->horizontal_accuracy); data_file.print(',');
  data_file.println(ubx_pos_llh->vertical_accuracy);
}

void LogGPSVelNED(void)
{
  UBXVelNED * ubx_vel_ned = reinterpret_cast<UBXVelNED *>(ublox_serial.Data());
  data_file.print("2,");

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

  data_file.print(ubx_sol->gps_ms_time_of_week); data_file.print(',');
  data_file.print(ubx_sol->fractional_time_of_week); data_file.print(',');
  data_file.print(ubx_sol->gps_week); data_file.print(',');
  data_file.print((int)ubx_sol->gps_fix_type); data_file.print(',');
  data_file.print((int)ubx_sol->gps_fix_status_flags); data_file.print(',');
  data_file.print(ubx_sol->ecef_x_coordinate); data_file.print(',');
  data_file.print(ubx_sol->ecef_y_coordinate); data_file.print(',');
  data_file.print(ubx_sol->ecef_z_coordinate); data_file.print(',');
  data_file.print(ubx_sol->coordinate_accuracy); data_file.print(',');
  data_file.print(ubx_sol->ecef_x_velocity); data_file.print(',');
  data_file.print(ubx_sol->ecef_y_velocity); data_file.print(',');
  data_file.print(ubx_sol->ecef_z_velocity); data_file.print(',');
  data_file.print(ubx_sol->velocity_accuracy); data_file.print(',');
  data_file.print(ubx_sol->position_dop); data_file.print(',');
  data_file.println((int)ubx_sol->number_of_satelites_used);
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

  // Initialize the SD card (fails if not present).
  // sd_initialized = sd.begin(CARD_SELECT, SPI_HALF_SPEED);

  // Debug output over USB programming port.
  Serial.begin(57600);

  // mk_serial.Init();
  ublox_serial.Init();
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
      char filename_char[10] = {"00000.csv"};
      String filename_string(millis() / 1000);
      filename_string.toCharArray(&filename_char[5 - filename_string.length()],
        filename_string.length() + 1);
      filename_char[5] = '.';
      data_file = sd.open(filename_char, FILE_WRITE);

      if (data_file)
      {
        // Request a data stream from the FlightCtrl
        logging_active = true;
        digitalWrite(GREEN_LED, HIGH);
        // Serial2.print("#0i?y\r");  // Request data stream (encoded).
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

    ublox_serial.Pop();
  }



/*
  mk_serial.ProcessIncoming();

  if (mk_serial.IsAvailable())
  {
    if (logging_active)
    {
      digitalWrite(LED2, HIGH);

      struct MKData {
        int16_t int16[8];
        uint8_t uint8[2];
      };
      MKData* packet = reinterpret_cast<MKData*>(mk_serial.Data());

      data_file.print(millis());
      data_file.print(",");
      for (int i = 0; i < 8; i++)
      {
        data_file.print(packet->int16[i]);
        data_file.print(",");
      }
      data_file.print((int)packet->uint8[0]);
      data_file.print(",");
      data_file.println((int)packet->uint8[1]);

      digitalWrite(LED2, LOW);
    }
    else
    {
      Serial2.print("#0r@B\r");  // Reset data stream (encoded).
    }
    mk_serial.Pop();
  }
*/
}
