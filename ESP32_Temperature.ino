#include <SPI.h>
#include <WiFi.h>
#include <SSD1306Wire.h>
#include <OneWire.h>
#include <MQTT.h>

// ***** NOTES *****

// Use board WEMOS LOLIN32 to compile
// Using MQTT from https://github.com/256dpi/arduino-mqtt

// Hardware attached: 
// Dallas One wire sensors (at pin ONEWIRE_PIN)
// Led connected through resistor to ground at pin LED_PORT
// Board: Wemos ESP32 with display (example https://www.banggood.com/Wemos-ESP32-OLED-Module-For-Arduino-ESP32-OLED-WiFi-Bluetooth-Dual-ESP-32-ESP-32S-ESP8266-p-1148119.html?rmmds=search&cur_warehouse=CN)

// ***** NOTES *****



#define ONEWIRE_PIN             14      // OneWire Dallas sensors are connected to this pin
#define MAX_NUMBER_OF_SENSORS   3       // maximum number of Dallas sensors
#define ADDRESS_LENGTH          8       // Length of onewire address

OneWire  ds(ONEWIRE_PIN);           	// (a 4.7K pull-up resistor is necessary)
SSD1306Wire  display(0x3c, 5, 4);
byte numberOfFoundSensors;

#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

const char mqtt_hostname[] = "IP_ADDRESS_TO_BROKER";
const int mqtt_port = 1883;
const char mqtt_user[] = "MQTT_USER_NAME";
const char mqtt_user_password[] = "MQTT_USER_PASSWORD";

#define LED_PORT 15
#define SHOW_DALLAS_ERROR


MQTTClient client;
WiFiClient net;
unsigned long lastMQTTPublishMillis = 0;

SemaphoreHandle_t displaySemaphore;
const TickType_t xDelay500ms = pdMS_TO_TICKS(500UL);

struct sensorStruct
{
    byte addr[ADDRESS_LENGTH];
    float temp;
    String name;
    String displayName;
} sensor[MAX_NUMBER_OF_SENSORS];

void DoLog(String a)
{
    int pad_length = 60 - a.length();
    String pad = "";
    for (int i = 0; i < pad_length; i++)
    {
        pad += " ";
    }
    Serial.println(a + pad + "[" + String((int)(millis())) + String(" msec]"));
}

void SetupDisplay() 
{
    display.init();
    display.flipScreenVertically();
    display.setColor(WHITE);
}

void ShowDisplayIntro()
{
    DoLog("Showing display intro...");
    display.clear();
    display.drawRect(0, 0, 128, 64);
    display.setTextAlignment(TEXT_ALIGN_CENTER);
    display.setFont(ArialMT_Plain_10);
    int start = 5; int space = 13;
    display.drawString(64, start, String("Temperature"));
    start += space;
    display.drawString(64, start, String("measurement"));
    start += space;
    display.drawString(64, start, String("RapidApps 2018"));
    start += space;
    display.drawString(64, start, String("Sensorpin: " + String(ONEWIRE_PIN)));
    display.display();
    DoLog("Done Showing display intro !!");
}

void AquireTemperatureFromSensorsTask(void * pvParameters)
{
    numberOfFoundSensors = 0;
    byte currentAddr[8];
    while (ds.search(currentAddr) && numberOfFoundSensors < MAX_NUMBER_OF_SENSORS)
    {
        for (byte i = 0; i < 8; i++)
        {
            sensor[numberOfFoundSensors].addr[i] = currentAddr[i];           
        }
        numberOfFoundSensors++;
    }
    DoLog(String(numberOfFoundSensors) + " Dallas sensors found.");

    if (!numberOfFoundSensors)
    {
        vTaskDelete(NULL);
    }
    else
    {
        for (byte thisSensor = 0; thisSensor < numberOfFoundSensors; thisSensor++)
        {
            sensor[thisSensor].displayName = GetDisplayNameForSensor(convertAddressToString(sensor[thisSensor]));
        }
    }
    /* Main task loop */
    while (1)
    {
        for (byte thisSensor = 0; thisSensor < numberOfFoundSensors; thisSensor++)
        {
            ds.reset();
            ds.select(sensor[thisSensor].addr);
            ds.write(0x44, 0); // Start conversion, with parasite power off at the end
        }

        vTaskDelay(750 / portTICK_PERIOD_MS); // Wait for conversion ready

        for (byte thisSensor = 0; thisSensor < numberOfFoundSensors; thisSensor++)
        {
            byte data[12];
            ds.reset();
            ds.select(sensor[thisSensor].addr);
            ds.write(0xBE);  // Read Scratchpad
            for (byte i = 0; i < 9; i++)
            { // we need 9 bytes
                data[i] = ds.read();
            }
            byte type_s;
            switch (sensor[thisSensor].addr[0])  // The first ROM byte indicates which chip
            {
            case 0x10:
                type_s = 1;
                sensor[thisSensor].name = "DS18S20";
                break;
            case 0x28:
                sensor[thisSensor].name = "DS18B20";
                type_s = 0;
                break;
            case 0x22:
                sensor[thisSensor].name = "DS1822";
                type_s = 0;
                break;
            default:
#ifdef SHOW_DALLAS_ERROR
                DoLog("Device is not a DS18x20 family device. [Devicetype : " + String(sensor[thisSensor].addr[0]) + "]");
#endif
                return;
            }

            int16_t raw;
            if (OneWire::crc8(data, 8) != data[8])
            {
#ifdef SHOW_DALLAS_ERROR
                // CRC of temperature reading indicates an error, so we print a error message and discard this reading
                DoLog("*** CRC error from device " + String(thisSensor) + " : " + sensor[thisSensor].displayName);
#endif
            }
            else
            {
                raw = (data[1] << 8) | data[0];
                if (type_s)
                {
                    raw = raw << 3; // 9 bit resolution default
                    if (data[7] == 0x10)
                    {
                        raw = (raw & 0xFFF0) + 12 - data[6]; // "count remain" gives full 12 bit resolution
                    }
                }
                else
                {
                    byte cfg = (data[4] & 0x60);
                    // at lower res, the low bits are undefined, so let's zero them
                    if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
                    else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
                    else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
                                                          //// default is 12 bit resolution, 750 ms conversion time
                }
                sensor[thisSensor].temp = raw / 16.0;
            }
        }
    }
}

String GetDisplayNameForSensor(String sensor_id)
{
    if (sensor_id.equals("286A42260000808D"))
        return "Sensor 1";
    else if (sensor_id.equals("28A3D63705000022"))
        return "Sensor 2";
    else
    {
        DoLog("*** No name defined for sensor '" + sensor_id + "'");
        return sensor_id;
    }
}

void ReconnectToMQTTIfNeeded()
{
    if (!client.connected())
    {
        ConnectToMQTT();
    }
}

void SetupMQTT()
{
    client.begin(mqtt_hostname, mqtt_port, net);
    client.onMessage(MQTTMessageReceived);
}

void RegisterMQTTSubscription()
{
    client.subscribe("led_status", 1);
}

void ConnectToMQTT()
{
    DoLog("Connecting to MQTT");
    while (!client.connect("ESP32_DuoTemp_Display", mqtt_user, mqtt_user_password)) {
        Serial.print(".");
        delay(500);
    }
    RegisterMQTTSubscription();
    DoLog("Connected to MQTT !!!");
}

void MQTTMessageReceived(String &topic, String &payload) {
    DoLog("======= >> MQTT incoming: " + topic + " - " + payload + ".");
    if (topic.equals("led_status"))
    {
        DoLog("Setting led to " + payload.equals("1") ? "On" : "Off");
        SetLedStatus(payload.equals("1"));
    }
    else
    {
        DoLog("No led command !");
    }
}

void PublishToMQTT() {
    DoLog("Publishing to MQTT...");
    ReconnectToMQTTIfNeeded();
    if (numberOfFoundSensors)
    {
        for (byte thisSensor = 0; thisSensor < numberOfFoundSensors; thisSensor++)
        {
            String topic = String("temp_") + String(thisSensor + 1);
            float temp = sensor[thisSensor].temp;
            String message = String(temp, '\001');
            client.publish(topic, message);
            DoLog("Published: " + topic + " -> "+ message + "   [Sensor : " + sensor[thisSensor].displayName + "]");
        }
    }
    DoLog("Done publishing to MQTT !!");
}

void ConnectToWifi()
{
    DoLog("Connecting to WiFi");
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(WiFiEvent);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    DoLog("WiFi connected !!");
}

void WiFiEvent(WiFiEvent_t event) {
    //Serial.printf("[WiFi-event] event: %d\n", event);
    switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        DoLog("WiFi connected. ");
        DoLog("IP address: ");
        DoLog(String(WiFi.localIP()));
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        DoLog("**** WiFi lost connection. ****");
        DoLog("Reconnect = " + WiFi.getAutoReconnect());
        break;
    }
}

void SetupLed() {
    pinMode(LED_PORT, OUTPUT);
}

void SetLedStatus(boolean on) {
    digitalWrite(LED_PORT, on ? HIGH : LOW);
}

void Initialize()
{
    Serial.begin(115200);
    DoLog("START");
    displaySemaphore = xSemaphoreCreateMutex();
    if (displaySemaphore == NULL)
    {
        DoLog("************* UNABLE TO AQUIRE SEMAPHORE !!!!");
    }
}


void DisplayTemperatureTask(void * pvParameters)
{
    unsigned int sensorIndexToShow = 0;
    if (!numberOfFoundSensors)
    {
        vTaskDelete(NULL);
    }
    else
    {
        while (1)
        {
            ShowTemperature(sensor[sensorIndexToShow]);
            sensorIndexToShow++;
            if (sensorIndexToShow >= numberOfFoundSensors)
            {
                sensorIndexToShow = 0;
            }
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
    }
}


void PublishToMQTTTask(void * pvParameters)
{
    if (!numberOfFoundSensors)
    {
        vTaskDelete(NULL);
    }
    else
    {
        while (1)
        {
            PublishToMQTT();
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }
}

void setup()
{
    Initialize();
    SetupDisplay();
    ShowDisplayIntro();
    SetupLed();

    ConnectToWifi();
    SetupMQTT();
    ConnectToMQTT();

    xTaskCreatePinnedToCore(
        AquireTemperatureFromSensorsTask,   /* Function to implement the task */
        "AquireTemp",                       /* Name of the task */
        4000,                               /* Stack size in words */
        NULL,                               /* Task input parameter */
        5,                                  /* Priority of the task */
        NULL,                               /* Task handle. */
        1);                                 /* Core where the task should run */


    xTaskCreatePinnedToCore(
        PublishToMQTTTask,                  /* Function to implement the task */
        "PublishToMQTT",                    /* Name of the task */
        4000,                               /* Stack size in words */
        NULL,                               /* Task input parameter */
        5,                                  /* Priority of the task */
        NULL,                               /* Task handle. */
        tskNO_AFFINITY);                    /* Core where the task should run */


    xTaskCreatePinnedToCore(
        DisplayTemperatureTask,             /* Function to implement the task */
        "DisplayTemp",                      /* Name of the task */
        1000,                               /* Stack size in words */
        NULL,                               /* Task input parameter */
        5,                                  /* Priority of the task */
        NULL,                               /* Task handle. */
        tskNO_AFFINITY);                    /* Core where the task should run */

    xTaskCreatePinnedToCore(
        DisplayTimeTask,                    /* Function to implement the task */
        "DisplayTime",                      /* Name of the task */
        1000,                               /* Stack size in words */
        NULL,                               /* Task input parameter */
        5,                                  /* Priority of the task */
        NULL,                               /* Task handle. */
        tskNO_AFFINITY);                    /* Core where the task should run */
}


String convertAddressToString(struct sensorStruct tempStruct)
{
    String result = "";
    for (int i = 0; i < ADDRESS_LENGTH; i++)
    {
        char temp[2];
        sprintf(temp, "%02X", tempStruct.addr[i]);
        result += String(temp);
    }
    return result;
}

void DisplayTimeTask(void * pvParameters)
{
    while (1)
    {
        showTime();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void ShowTemperature(struct sensorStruct tempStruct)
{
    if (xSemaphoreTake(displaySemaphore, xDelay500ms) == pdPASS)
    {
        display.clear();
        display.setColor(WHITE);
        display.drawRect(0, 0, 128, 64);
        display.setTextAlignment(TEXT_ALIGN_CENTER);
        display.setFont(ArialMT_Plain_24);
        display.drawString(64, 3, String(tempStruct.temp) + String(" °C"));
        display.setFont(ArialMT_Plain_16);
        display.drawString(64, 30, tempStruct.displayName);        
        xSemaphoreGive(displaySemaphore);
    }
    showTime();
}

void showTime()
{
    if (xSemaphoreTake(displaySemaphore, xDelay500ms) == pdPASS)
    {
        display.setColor(BLACK);
        display.fillRect(1, 49, 126, 10);
        display.setColor(WHITE);
        display.setFont(ArialMT_Plain_10);
        display.drawString(64, 49, String((int)(millis() / 1000.0)) + String(" sec"));
        display.display();
        xSemaphoreGive(displaySemaphore);
    }
}

void loop()
{
    client.loop();
}


