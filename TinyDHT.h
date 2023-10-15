/*!
 * @file TinyDHT.h
 */
#ifndef DHT_H
#define DHT_H
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Tiny DHT library
Uses integer math to save space on Trinket/Gemma

MIT license
written by Adafruit Industries
*/

#define MAXTIMINGS                                                             \
  85 //!< how many timing transitions we need to keep track of. 2 * number bits
     //!< + extra

#define DHT11 11  //!< Used to specify that you want to use the DHT11
#define DHT22 22  //!< Used to specify that you want to use the DHT22
#define DHT21 21  //!< Used to specify that you want to use the DHT21
#define AM2301 21 //!< Used to specify that you want to use the AM2301

// NAN code in DHT library takes space, define bad values here
#define BAD_HUM -1    //!< Bad humitidy reading
#define BAD_TEMP -999 //!< Bad temperature reading

/*!
 * @brief Class that stores the state and functions for the DHT
 */
class DHT {
private:
  uint8_t data[6];
  uint8_t _pin, _type;
#ifdef __AVR
  // Use direct GPIO access on an 8-bit AVR so keep track of the port and
  // bitmask for the digital pin connected to the DHT.  Other platforms will use
  // digitalRead.
  uint8_t _bit, _port;
#endif
  unsigned long _lastreadtime, _maxcycles;
  bool firstreading;

  bool read(void);
  uint32_t expectPulse(bool level);

public:
  /*!
   * @brief DHT constructor
   */
  DHT();

  /*!
   * @brief Begins connection with device
   * @param pin Pin connected to the DHT
   * @param type What sensor you're connecting, DHT11, DHT22, DHT21, AM2301
   */
  void begin(uint8_t pin, uint8_t type);

  /*!
   * @brief Reads the temperature from device
   * @return Returns the temperature
   */
  int16_t readTemperature(void);

  /*!
   * @brief Reads the humidity from the device
   * @return Returns the humidity read from the device
   */
  uint8_t readHumidity(void);
};

/*!
 *  @brief  Class that defines Interrupt Lock Avaiability
 */
class InterruptLock {
public:
  InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    noInterrupts();
#endif
  }
  ~InterruptLock() {
#if !defined(ARDUINO_ARCH_NRF52)
    interrupts();
#endif
  }
};

#endif
