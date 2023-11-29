/*
 * An device that united an dht11 sensor and switch
 * (in my case switch controll an relay with connected floor lamp, and monitor climat in room )
 * DHT lib: https://github.com/adafruit/DHT-sensor-library
 * homekit lib: https://github.com/Mixiaoxiao/Arduino-HomeKit-ESP8266
*/

#include "DHT.h"
#include <Arduino.h>
#include <arduino_homekit_server.h>
#include "wifi_info.h"

#define DHTPIN 2
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

#define LOG_D(fmt, ...)   printf_P(PSTR(fmt "\n") , ##__VA_ARGS__);

void setup() {
	Serial.begin(115200);
  dht.begin();
	wifi_connect(); // in wifi_info.h
	// homekit_storage_reset(); // to remove the previous HomeKit pairing storage when you first run this new HomeKit example
	my_homekit_setup();
}

void loop() {
	my_homekit_loop();
	delay(10);
}

//==============================
// HomeKit setup and loop
//==============================

// access your HomeKit characteristics defined in my_accessory.c
extern "C" homekit_server_config_t config;
extern "C" homekit_characteristic_t cha_switch_on;

extern "C" homekit_characteristic_t cha_temperature;
extern "C" homekit_characteristic_t cha_humidity;
extern "C" homekit_characteristic_t cha_heat_index;

static uint32_t next_heap_millis = 0;
static uint32_t next_report_millis = 0;

#define PIN_SWITCH D1

//Called when the switch value is changed by iOS Home APP
void cha_switch_on_setter(const homekit_value_t value) {
	bool on = value.bool_value;
	cha_switch_on.value.bool_value = on;	//sync the value
	LOG_D("Switch: %s", on ? "ON" : "OFF");
	digitalWrite(PIN_SWITCH, on ? LOW : HIGH);
}

void my_homekit_setup() {
	pinMode(PIN_SWITCH, OUTPUT);
	digitalWrite(PIN_SWITCH, HIGH);

	//Add the .setter function to get the switch-event sent from iOS Home APP.
	//The .setter should be added before arduino_homekit_setup.
	//HomeKit sever uses the .setter_ex internally, see homekit_accessories_init function.
	//Maybe this is a legacy design issue in the original esp-homekit library,
	//and I have no reason to modify this "feature".
	cha_switch_on.setter = cha_switch_on_setter;
	arduino_homekit_setup(&config);

	//report the switch value to HomeKit if it is changed (e.g. by a physical button)
	//bool switch_is_on = true/false;
	//cha_switch_on.value.bool_value = switch_is_on;
	//homekit_characteristic_notify(&cha_switch_on, cha_switch_on.value);
}

void my_homekit_report() {
	float temperature = dht.readTemperature();
	float humidity = dht.readHumidity();
  float heat_index = dht.computeHeatIndex(temperature, humidity, false);

  cha_temperature.value.float_value = temperature;
	homekit_characteristic_notify(&cha_temperature, cha_temperature.value);

	cha_humidity.value.float_value = humidity;
	homekit_characteristic_notify(&cha_humidity, cha_humidity.value);

  cha_heat_index.value.float_value = heat_index;
	homekit_characteristic_notify(&cha_heat_index, cha_heat_index.value);

	LOG_D("temperature: %.1f, humidity: %.1f, heat_index: %1.f", temperature,humidity,heat_index);
}

void my_homekit_loop() {
	arduino_homekit_loop();
	const uint32_t t = millis();
	if (t > next_report_millis) {
		// report sensor values every 10 seconds
		next_report_millis = t + 10 * 1000;
		my_homekit_report();
	}
	if (t > next_heap_millis) {
		// Show heap info every 5 seconds
		next_heap_millis = t + 5 * 1000;
		LOG_D("Free heap: %d, HomeKit clients: %d",
				ESP.getFreeHeap(), arduino_homekit_connected_clients_count());

	}
}
