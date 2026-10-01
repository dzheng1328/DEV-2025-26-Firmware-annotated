#include "vehicle.hpp" //"vehicle.hpp" because relative to this project's own files
#include <stdint.h>

#if defined(ARDUINO_ARCH_ESP32)
#include <ESP32-TWAI-CAN.hpp>
#ifndef VEHICLE_TWAI_TX_PIN 
#define VEHICLE_TWAI_TX_PIN 33  //if not defined, default to pin 33
#endif
#ifndef VEHICLE_TWAI_RX_PIN
#define VEHICLE_TWAI_RX_PIN 34  //if not defined, default to pin 34
#endif
#elif defined(CORE_TEENSY)
#include <FlexCAN_T4.h>
#else
#error "CAN network library not written for chosen architecture"
#endif // defined

Vehicle g_vehicle; //definition matching hpp extern declaration.
                    //this line is what actually reserves memory for the object, other file borrows a reference to this

#if defined(CORE_TEENSY)
static FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> s_teensy_can; //FlexCAN_T4<...> is a template
                                                               //generic over some choices like which CAN peripheral, how big receive/transmit buffers are
//static here means something else than static method header, "only visible within this cpp file, not linkable from anywhere else"
#endif

#if defined(ARDUINO_ARCH_ESP32)
void Vehicle::init_network(DevBoard board) { //Vehicle:: is the scope resolution operator "this is the definition of the init_network" function that was declared as a member of Vehicle back in the header
                                             //every member function's body uses this ClassName:: prefix when defined outside class body
  m_board = board;
  ESP32Can.setPins(VEHICLE_TWAI_TX_PIN, VEHICLE_TWAI_RX_PIN);
  ESP32Can.setTxQueueSize(10);
  m_last_error = ESP32Can.begin(ESP32Can.convertSpeed(500)) ? ESP_OK : ESP_FAIL; //? : ternary operator, if true ok if no fail
  if (m_last_error != ESP_OK) {
    return; //if not okay (fail) bare return exits init_network immediately if setup failed
  }

  xTaskCreate(Vehicle::twai_receive_task, "twai_rx", 4096, this, 5, nullptr); //hands the FreeRTOS schedule a new background task to run forever in parallel w/ main code
                                                                              //Vehicle::twai_receive_task no () so this is passing function itself, not calling it 
                                                                              //name for debugging "twai_rx"
                                                                              //stack size 4096 bytes
                                                                              //argument to hand that function when it starts (this, current Vehicle object)
                                                                                //this is why twai_receive_task had to take a void *args parameter: FreeRTOS API doesn't know about Vehicle class, so it can only pass a generic pointer
                                                                                //this is smuggled through as that generic pointer, with priority (5)
                                                                              //place to optionally store a handle to the created task (nullptr)
                                                                      
}

void Vehicle::send_message(uint32_t id, uint8_t len, const uint8_t *data) { 
  CanFrame frame = {0}; // = {0} zero-initializes every field of frame (struct from ESP32 CAN lib) matters because struct values would default to garbage values from previous ops
  //uninitialized extd or data_length_code field could send a malformed frame
  frame.identifier       = id;
  frame.extd             = 0;
  frame.data_length_code = len;

  memcpy(frame.data, data, len); //memcpy copies raw bytes from one place to another without understanding what those bytes mean
  //necessary here instead of frame.data = data because frame.data and data are two DIFF arrays living in 2 diff addresses, assignment doesn't exist for raw C-style arrays it does for oop

  bool ok = ESP32Can.writeFrame(frame);
}

void Vehicle::twai_receive_task(void *args) { //other half of the xTaskCreate call above
                                              //this static function receives back the generic void *args FreeRTOS handed it
                                              //immediately converts back into a real Vehicle * with static_cast<Vehicle*> (explicit, compile-time checked cast)
  Vehicle *self = static_cast<Vehicle*>(args);
  CanFrame frame;
  while (true) { //infinite loop
      if (ESP32Can.readFrame(frame, 100)) { //if the can frame reads 
          uint8_t data[8]; //declares a local, stack-allocated 8 byte array EVERY TIME a frame arrives
                           //this is fine because stack-allocated arrays are instant and automatically reclaimed the moment this loop iteration ends
          memcpy(data, frame.data, 8); //copy data (destination), frame.data (source), 8 (# byes)
          self->on_receive(frame.identifier, frame.data_length_code, data); //calls the (non-static, needs specific object) on_receive member function ON the object self points to
                                                                            //-> is the "dereference then access a member" operator, shorthand for (*self).on_receive(...)
      }
  }
}
#endif // defined(ARDUINO_ARCH_ESP32)

#if defined(CORE_TEENSY) //Teensy version of foward_flexcan does the exact same job as the above ver for ESP32 because the two underlying libs have diff callback API
                         //FreeRTOS xTaskCreate inssts on void(*)(void*) shaped function, forcing static_cast round trip
                         //Teensy library's onReceive callback doesn't take a user arg at all, forward_flexcan below has to reach the single global g_vehicle directly instead of receiving "which vehicle" as a param

void Vehicle::forward_flexcan(const CAN_message_t &msg) { //simpler than the ESP32 path precisely because of the difference explained above
                                                          //this callback just reaches directly for one global g_vehicle instead of receiving it as an argument
  g_vehicle.on_receive(msg.id, msg.len, msg.buf);
}

void Vehicle::init_network(DevBoard board) { //init
  m_board = board;
  s_teensy_can.begin();
  s_teensy_can.setBaudRate(500000);
  s_teensy_can.enableFIFO(true);
  s_teensy_can.enableFIFOInterrupt();
  s_teensy_can.onReceive(forward_flexcan);
}

void Vehicle::send_message(uint32_t id, uint8_t len, const uint8_t *data) {
  // TODO: align with ESP32 implementation once message layout exists
  CAN_message_t msg;
  msg.id = id;
  msg.len = len;
  msg.flags.extended = 0;  // Use standard 11-bit IDs (set to 1 for 29-bit extended)
  msg.flags.remote = 0;    // Not a remote frame
  msg.seq = 0;             // Not using sequential mode

  memcpy(msg.buf, data, len);

  s_teensy_can.write(msg);
}
#endif // defined(CORE_TEENSY)


void Vehicle::send_all() { //bug notice for how we fill data out in the switch cases:
                           //data is a plain stack array with no ={0} initializer so it starts as whatever bits were already sitting at that stack address
                           //leftover from some earlier function call, possibly the previous send_all() call if nothing else reused that stack slot in betwee.
                           //not guaranteed to be zero.

                           //so each case has garbage bytes:
                            //i.e. peripherals: bytes written: data[0]->data[5], bytes sent (len), garbage bytes data[6], data[7]
                            //... all the way down
                          //every board's heartbeat frame has 7 garbage trailing bytes 

                          //potential harm cases:
                            //undefined behavior, not just harmless zeros; relying on uninitialized memory happening to look sane is dangerous, works by accident of current stack layout, but not by guarantee
                            //bus loading: sending a full 8-byte dlc when only 4 bytes are meaningful wastes bus bandwidth
                              //especially since CAN supports a shorter DLC and send_message's len parameter could just be passed as 4 instead of hardcoded 8
                            //debugging hazard: if we ever put a CAN analyzer on the bus, those trailing bytes will show up like real, changing data if the'yre not literally zero
                            //latent bug for future code: if someone later adds signal that needs those trailing bytes without explicitly zero-initializing first, they'd inherit garbage in the bytes they don't touch on the same call


  uint8_t data[8]; //8byte array
  uint8_t heartbeat[8]; //8byte array
  heartbeat[0] = 1; //set msb to 1
  switch (m_board) { //branches based on which board role this specific Vehicle instance was initialized as
                     //each board only ever sends the messages relevant to itself
  case PERIPHERALS: //matches when m_board == DevBoard::PERIPHERALS;

  //BUG!!!
  //before a specific commit, every case in send_all() started with
  //uint8_t* data = new uint8_t[8];

  //instead of today's uint8_t data[8] defined before the switch
  //new uint8_t[8] asks the HEAP for 8 bytes at runtime and returns a pointer to them, and critically, memory obtained with new in c++ is never automatically reclaimed
  //it must be matched with an explicit delete[] data, which was not defined anywhere
  //every single call to send_all() permanently lost 8 bytes of heap memory forever.
    data[0] = g_vehicle.m_peripherals_windshield;
    data[1] = g_vehicle.m_peripherals_backrunninglights;
    data[2] = g_vehicle.m_peripherals_turn;
    data[3] = g_vehicle.m_peripherals_headlights;
    data[4] = g_vehicle.m_peripherals_brakelights;
    data[5] = g_vehicle.m_peripherals_hazard;

    g_vehicle.send_message(0x100, 8, data); //0x100 is a hexadecimal literal (0x prefix)
                                            //this is the CAN arbitration ID for this message
                                            //notice peripherals uses 0x100 while motor controller uses 0x300
                                            //motor data always wins bus contention over peripheral data

    g_vehicle.send_message(0x101, 8, heartbeat);//every board also sends a second heartbeat message right after its data message, at id+1, simple "im alive" signal
    break; //break; at the end of each case stops execution from "falling through" into the next case
  
  case POWER_DISTRIBUTION: //byte-packing: m_pdb_current is a uint16_t (16 bits, 2 byes), but each element of data[] is only a uint8*t (8 bits, 1 byte) 
                           //a 16-bit cvalue has to be split across two bytes to go into the CAN payload array
    data[0] = (g_vehicle.m_pdb_current >> 8) & 0xFF; // >> 8 shifts every bit 8 positions to the right, which moves the UPPER (more significant) byte down into the lower 8 bits, discarding what falls off to the bottom
    data[1] = g_vehicle.m_pdb_current & 0xFF; // &0xFF (0xFF = binary 00000000 11111111, all 8 bits set) then masks that down to just the lowest byte, in case any stray bits remain above bit 7 after the shift
                                              // so data[0] gets the high byte, and data[1] = g_vehicle.m_pdf_current & 0xFF (no shoft needed) directly masks out the LOW byte
    data[2] = (g_vehicle.m_pdb_voltage >> 8) & 0xFF; //this exact pattern is repeated for every multi-byte value in this function
                                                     //including the 4-byte m_motor_rpm split across data[0...3] using shifts of 24, 16, 8, and 0
    data[3] = g_vehicle.m_pdb_voltage & 0xFF; 
    data[4] = g_vehicle.m_pdb_motor_enabled; //just an 8bit value, no manipulation needed here
    
    g_vehicle.send_message(0x200, 8, data);  //send message for pdb serial 0x200, with 8byte value , data being the array
    g_vehicle.send_message(0x201, 8, heartbeat);
    break;

  case MOTOR_CONTROLLER: //same logic, the big shift
    data[0] = (g_vehicle.m_motor_rpm >> 24) & 0xFF;
    data[1] = (g_vehicle.m_motor_rpm >> 16) & 0xFF;
    data[2] = (g_vehicle.m_motor_rpm >> 8) & 0xFF;
    data[3] = g_vehicle.m_motor_rpm & 0xFF;

    g_vehicle.send_message(0x300, 8, data); //serial for motor: 0x300
    g_vehicle.send_message(0x301, 8, heartbeat);
    break;
    
  case THROTTLE:
    data[0] = (g_vehicle.m_throttle_raw >> 8) & 0xFF; //same logic
    data[1] = g_vehicle.m_throttle_raw & 0xFF;
    data[2] = (g_vehicle.m_throttle_average >> 8) & 0xFF;
    data[3] = g_vehicle.m_throttle_average & 0xFF;

    g_vehicle.send_message(0x400, 8, data); //serial for throttle: 0x400
    g_vehicle.send_message(0x401, 8, heartbeat);
    break;
    
  case JOULEMETER:
    data[0] = (g_vehicle.m_joulemeter_current >> 8) & 0xFF; //same logic
    data[1] = g_vehicle.m_joulemeter_current & 0xFF;
    data[2] = (g_vehicle.m_joulemeter_voltage >> 8) & 0xFF;
    data[3] = g_vehicle.m_joulemeter_voltage & 0xFF;
    data[4] = (g_vehicle.m_joulemeter_energy >> 24) & 0xFF;
    data[5] = (g_vehicle.m_joulemeter_energy >> 16) & 0xFF;
    data[6] = (g_vehicle.m_joulemeter_energy >> 8) & 0xFF;
    data[7] = g_vehicle.m_joulemeter_energy & 0xFF;
    
    g_vehicle.send_message(0x500, 8, data); //serial for joulemeter: 0x500
    g_vehicle.send_message(0x501, 8, heartbeat);
    break;

  case COMMUNICATIONS:
    g_vehicle.send_message(0x600, 8, heartbeat); //serial for comms: 0x600
    break;

  case TEST_BOARD:
    g_vehicle.send_message(0x700, 8, heartbeat); //serial for test board: 0x700
    break;

  default:
    // should never reach here
    break;
  } //notice how the switch content is ordered from less to more important due to CAN message arbitration, test board just added for sit&gigs
    //honestly could've ordered these better, but idk if the group knows abt this
}

void Vehicle::on_receive(uint32_t id, uint8_t len, const uint8_t *data) { //the exact reverse of packing
  switch (id) {
  case 0x100:
    g_vehicle.m_peripherals_windshield = data[0]; //nice here so every byte is meant for a different message abut each peripheral
    g_vehicle.m_peripherals_backrunninglights = data[1];
    g_vehicle.m_peripherals_turn = data[2];
    g_vehicle.m_peripherals_headlights = data[3];
    g_vehicle.m_peripherals_brakelights = data[4];
    g_vehicle.m_peripherals_hazard = data[5];
    break;
  case 0x101:
    // Contains software versions, not useful
    break;
  case 0x200:
    g_vehicle.m_pdb_current = (data[0] << 8) | data[1]; //data[0] << 8 shifts the high byte back up into its original bit position
                                                        //undoing the earlier >> 8, then data[1] merges in the low byte alongside it
                                                        //or works here specifically because the two operands occupy entirely non-overlapping bit positions (bits 8-15) from shifted data[0], bits 0-7 from data[1]
    g_vehicle.m_pdb_voltage = (data[2] << 8) | data[3]; //same for this, data[2] + data[3] concatenated to 2byte 
    g_vehicle.m_pdb_motor_enabled = data[4]; //only 1byte needed 
    break;
  case 0x201://heartbeats, not read
    // Contains software versions, not useful
    break;
  case 0x300:
    g_vehicle.m_motor_rpm = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3]);
    break;
  case 0x301:
    // Contains software versions, not useful
    break;
  case 0x400:
    g_vehicle.m_throttle_raw = (data[0] << 8) | data[1];
    g_vehicle.m_throttle_average = (data[2] << 8) | data[3];
    break;
  case 0x401:
    // Contains software versions, not useful
    break;
  case 0x500:
    g_vehicle.m_joulemeter_current = (data[0] << 8) | data[1];
    g_vehicle.m_joulemeter_voltage = (data[2] << 8) | data[3];
    g_vehicle.m_joulemeter_energy = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | (data[7]);
    break;
  case 0x501:
    // Contains software versions, not useful
    break;
  case 0x600:
    // Contains software versions, not useful
    break;
  default:
    break;
  }
}
