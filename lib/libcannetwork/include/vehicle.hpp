/* header file
declares what exists i.e. Vehicle class, member variables, signatures of functions 
without saying how each function works 

vehicle.cpp is the source, provides actual function bodies */

#pragma once //compiler instruction, telling compiler "if this file gets included more than once while building, only include it the first time"
#include <stdint.h> // #include basically pastes the entire contents of another file in right at this line
                    // before compilation starts 

/* this whole block runs before compilation, based on the board we're building for
defined(X) asks if the macro X has been defined, and if so gets kept, so can either compile diff for esp32 vs teensy*/
#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32-TWAI-CAN.hpp>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#elif defined(CORE_TEENSY)
#include <FlexCAN_T4.h>
#else
#error "CAN network library not written for chosen architecture" //if none, error out
#endif // defined

enum DevBoard { //enum creates a new type whose only legal values are in the list
                // name is an integer, assigned in order starting from 0, but writing DevBoard::THROTTLE is more readable instead of 4
  COMMUNICATIONS,
  PERIPHERALS,
  MOTOR_CONTROLLER,
  POWER_DISTRIBUTION,
  THROTTLE,
  JOULEMETER,
  TEST_BOARD,
  // Number of designs this library is using
  BOARD_COUNT,
};

class Vehicle { //class bundles data and functions together
private:
  DevBoard m_board; //private because nothing outside this class should be changing which board this instance thinks it is 

#if defined(ARDUINO_ARCH_ESP32)
  
#elif defined(CORE_TEENSY)
  // Teensy uses the shared static instance in the source file.
#else
#error "CAN network library not written for chosen architecture"
#endif

public:
  uint8_t m_peripherals_windshield; //all the m_ prefixes are a naming convention, meaning "this is a member variable of the class" so that inside a function body it's visually obvious whether a name refers to a class member or a local variable
  uint8_t m_peripherals_backrunninglights;
  uint8_t m_peripherals_turn;
  uint8_t m_peripherals_headlights;
  uint8_t m_peripherals_brakelights;
  uint8_t m_peripherals_hazard;

  uint16_t m_pdb_current;
  uint16_t m_pdb_voltage;
  uint8_t  m_pdb_motor_enabled;
  
  uint32_t m_motor_rpm;
  
  uint16_t m_throttle_raw;
  uint16_t m_throttle_average;
  
  uint16_t m_joulemeter_current;
  uint16_t m_joulemeter_voltage;
  uint32_t m_joulemeter_energy;
  // Private methods unique to architecture for sending/receiving can messages
private:
#if defined(ARDUINO_ARCH_ESP32)
  static void twai_receive_task(void *args); //
#elif defined(CORE_TEENSY)
  static void forward_flexcan(const CAN_message_t &msg); //Passing by reference here avoids copying the whole message struct just to read it.
#else
#error "CAN network library not written for chosen architecture"
#endif

  // Private methods common between architectures
private:
  void send_message(uint32_t id, uint8_t len, const uint8_t *data); //C/C++ arrays don't know their own length once decayed to a pointer,
                                                                    //so the length has to be tracked and passed alongside, by hand, every time.
  void on_receive(uint32_t id, uint8_t len, const uint8_t *data);

  // Public entry point
public:
  void init_network(DevBoard board);
  void send_all();
  #if defined(ARDUINO_ARCH_ESP32)
  esp_err_t m_last_error;
  #endif
};

extern Vehicle g_vehicle; //extern just says that a global variable of name g_vehicle exists somewhere, just promises it so that 
                          //any file that includes this header can refer to it
                          //is is why g_vehicle.m_throttle_raw = ... in boards/throttle/src/main.cpp and g_vehicle.send_all() are both operating on the exact same object.
