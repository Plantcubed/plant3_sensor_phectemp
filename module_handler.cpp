/** 
 *  \file module_handler.cpp
 *  \brief Handles all module integration. 
 *  \details See module_handler.h for details.
 *  \author Jake Rye
 */
#include "module_handler.h"

// Include Module Libraries
#include "communication.h"
#include "sensor_tsl2561.h"
#include "sensor_dht22.h"
#include "sensor_gc0011.h"
#include "actuator_relay.h"
#include "plant3_illum_panel.h"
#include "plant3_actuator_A3909Ghbridge.h"
#include "plant3_sensor_digitalfluidlevel.h"
#include "plant3_sensor_analogfluidlevel.h"
#include "plant3_sensor_phectemp.h"

#define RGB_PER_PANEL   107
#define LIGHT_LANELS    1

int EE_Address;

// Declare Module Objects
Communication communication;
SensorTsl2561 sensor_tsl2561_light_intensity_default("SLIN", 1, "SLPA", 1);
Plant3SensorPhECTemp sensor_water_ph_temperature_ec_default(A1, "SWPH", 1, 5, "SWTM", 1, A2, 6, "SWEC", 1);
SensorDht22 sensor_dht22_air_temperature_humidity_default(A0, "SATM", 1, "SAHU", 1);
SensorGc0011 sensor_gc0011_air_co2_temperature_humidity_default(12, 11, "SACO", 1, "SATM", 2, "SAHU", 2);
ActuatorRelay actuator_relay_air_heater_default(7, "AAHE", 1); // AC port 4
Plant3_Illum_Panel chamber_illumination_default("AUVI", 1, 18, "ALPN", 1, 8, RGB_PER_PANEL * LIGHT_LANELS, "SGWO", 1, 3);
ActuatorRelay actuator_relay_air_humidifier_default(9, "AAHU", 1); // AC port 1
ActuatorRelay actuator_relay_air_vent_default(14, "AAVE", 1);
ActuatorRelay actuator_relay_air_circulation_default(15, "AACR", 1);
Plant3_ActuatorA3909GHBridge dose_pump1_default("ADSP", 1, 38, 39, 1, 0);
Plant3_ActuatorA3909GHBridge dose_pump2_default("ADSP", 2, 40, 41, 1, 0);
Plant3_ActuatorA3909GHBridge dose_pump3_default("ADSP", 3, 42, 43, 1, 0);
Plant3_ActuatorA3909GHBridge dose_pump4_default("ADSP", 4, 44, 45, 1, 0);
ActuatorRelay actuator_relay_airpump_default(36, "AAPR", 1);
ActuatorRelay actuator_relay_co2_default(46, "ACOR", 1);
ActuatorRelay actuator_relay_watercir_default(33, "AWCR", 1);
ActuatorRelay actuator_relay_coolvalve_default(1, "ACOO", 1);

void initializeModules(void) {
	EE_Address = 0;
	communication.begin();
	sensor_water_ph_temperature_ec_default.begin();
	sensor_tsl2561_light_intensity_default.begin();
	sensor_dht22_air_temperature_humidity_default.begin();
	sensor_gc0011_air_co2_temperature_humidity_default.begin();
	actuator_relay_air_heater_default.begin();
	chamber_illumination_default.begin();
	actuator_relay_air_humidifier_default.begin();
	actuator_relay_air_vent_default.begin();
	actuator_relay_air_circulation_default.begin();
	dose_pump1_default.begin();
	dose_pump2_default.begin();
	dose_pump3_default.begin();
	dose_pump4_default.begin();
	actuator_relay_airpump_default.begin();
	actuator_relay_co2_default.begin();
	actuator_relay_watercir_default.begin();
	actuator_relay_coolvalve_default.begin();

	// Set Default States
	actuator_relay_air_circulation_default.set("AACR", 1, "1");
	actuator_relay_air_vent_default.set("AAVE", 1, "1");
	chamber_illumination_default.set("AWHI", 1, "0");
	chamber_illumination_default.set("AUVI", 1, "0");
	chamber_illumination_default.set("ARGB", 1, "0,0,0");
	dose_pump1_default.set("ADSP", 1, "0");
	dose_pump2_default.set("ADSP", 2, "0");
	dose_pump3_default.set("ADSP", 3, "0");
	dose_pump4_default.set("ADSP", 4, "0");
	actuator_relay_airpump_default.set("AAPR", 1, "0");
	actuator_relay_co2_default.set("ACOR", 1, "0");
	actuator_relay_watercir_default.set("AWCR", 1, "1");
	actuator_relay_coolvalve_default.set("AAPR", 1, "1");
}

void updateIncomingMessage(void) {
  // Check for Message(s) And Handle If Necessary
  String response_message = "";
  while (communication.available()) { // read in message(s) until nothing in serial buffer
    response_message += handleIncomingMessage();
  }
  // Append Responses From Message(s) Then Send
  if (response_message != "") {
    response_message = "\"GTYP\":\"Response\"," + response_message;
    response_message += "\"GEND\":0";
    communication.send(response_message);
  }
}

void updateStreamMessage(void) {
  // Initialize Stream Message
  String stream_message = "\"GTYP\":\"Stream\",";

  // Get Stream Message
  stream_message += sensor_water_ph_temperature_ec_default.get();
  stream_message += sensor_tsl2561_light_intensity_default.get();
  stream_message += sensor_dht22_air_temperature_humidity_default.get(); // does not work on 1.0
  stream_message += sensor_gc0011_air_co2_temperature_humidity_default.get();
  stream_message += actuator_relay_air_heater_default.get();
  stream_message += actuator_relay_air_humidifier_default.get();
  stream_message += actuator_relay_air_vent_default.get();
  stream_message += actuator_relay_air_circulation_default.get();
  stream_message += chamber_illumination_default.get();
  stream_message += dose_pump1_default.get();
  stream_message += dose_pump2_default.get();
  stream_message += dose_pump3_default.get();
  stream_message += dose_pump4_default.get();
  stream_message += actuator_relay_airpump_default.get();
  stream_message += actuator_relay_co2_default.get();
  stream_message += actuator_relay_watercir_default.get();
  stream_message += actuator_relay_coolvalve_default.get();

  // Return Stream Message
  stream_message += "\"GEND\":0";

  // Send Stream Message
  communication.send(stream_message);
}

String handleIncomingMessage(void) {
  // Parse Message into: Instruction Code - ID - Parameter
  String return_message = "";
  String incoming_message = communication.receive();
  Instruction instruction = parseIncomingMessage(incoming_message);

  // Pass Parsed Message To All Objects and Update Return Message if Applicable
  if (instruction.valid) {
	return_message += sensor_water_ph_temperature_ec_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += sensor_tsl2561_light_intensity_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += sensor_dht22_air_temperature_humidity_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += sensor_gc0011_air_co2_temperature_humidity_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += actuator_relay_air_heater_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += actuator_relay_air_humidifier_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += actuator_relay_air_vent_default.set(instruction.code, instruction.id, instruction.parameter);
    return_message += actuator_relay_air_circulation_default.set(instruction.code, instruction.id, instruction.parameter);
	return_message += chamber_illumination_default.set(instruction.code, instruction.id, instruction.parameter);
	return_message += dose_pump1_default.set(instruction.code, instruction.id, instruction.parameter);
	return_message += dose_pump2_default.set(instruction.code, instruction.id, instruction.parameter);
	return_message += dose_pump3_default.set(instruction.code, instruction.id, instruction.parameter);
	return_message += dose_pump4_default.set(instruction.code, instruction.id, instruction.parameter);
  }

  return return_message;
}

Instruction parseIncomingMessage(String message) {
  // Initialize Instruction
  Instruction instruction;
  instruction.valid = 0;

  // Get Instruction Data
  int len = message.length();
  int first_space = message.indexOf(" ");
  if ((first_space > 0) && (len > first_space)) {
    int second_space = message.indexOf(" ", first_space + 1);
    if ((second_space > 0) && (second_space < len - 1)) {
      // Received good message
      instruction.code = message.substring(0, 4);
      instruction.id = (message.substring(first_space, second_space)).toInt();
      instruction.parameter = message.substring(second_space + 1, len);
      instruction.valid = 1;
    }
  }

  // Return Instruction Data
  return instruction;
}

int Get_EE_AddressPtr(void)
{
	return EE_Address;
}

void Set_EE_AddressPtr(int NewAddress)
{
	EE_Address = NewAddress;
}

