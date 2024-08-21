// ESP32 Weather Display and a LilyGo EPD 4.7" Display, obtains Open Weather Map data, decodes and then displays it.
// This software, the ideas and concepts is Copyright (c) David Bird 2021. All rights to this software are reserved.
// #################################################################################################################

#include <Arduino.h>           // In-built
#include <esp_task_wdt.h>      // In-built
#include "freertos/FreeRTOS.h" // In-built
#include "freertos/task.h"     // In-built
#include "epd_driver.h"        // https://github.com/Xinyuan-LilyGO/LilyGo-EPD47
#include "esp_adc_cal.h"       // In-built

#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson
#include <HTTPClient.h>  // In-built
#include <UrlEncode.h>   // https://github.com/plageoj/urlencode

#include <WiFi.h> // In-built
#include <SPI.h>  // In-built
#include <time.h> // In-built
#include "SPIFFS.h"
#include "FS.h"

#include "owm_credentials.h"
#include "forecast_record.h"
#include "lang.h"
#include "web.h"

#define SCREEN_WIDTH EPD_WIDTH
#define SCREEN_HEIGHT EPD_HEIGHT

//################  VERSION  ##################################################
String version = "2.5 / 4.7in"; // Programme version, see change log at end
//################ VARIABLES ##################################################

enum alignment
{
    LEFT,
    RIGHT,
    CENTER
};

#define White     0xFF
#define LightGrey 0xBB
#define Grey      0x88
#define DarkGrey  0x44
#define Black     0x00

#define autoscale_on  true
#define autoscale_off false
#define barchart_on   true
#define barchart_off  false

boolean LargeIcon = true;
boolean SmallIcon = false;
#define Large 20 // For icon drawing
#define Small 8  // For icon drawing
String Time_str = "--:--:--";
String Date_str = "-- --- ----";
int wifi_signal, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0, EventCnt = 0, vref = 1100;
//################ PROGRAM VARIABLES and OBJECTS ##########################################
#define max_readings 24 // Limited to 3-days here, but could go to 5-days = 40

Forecast_record_type WxConditions[1];
Forecast_record_type WxForecast[max_readings];

float pressure_readings[max_readings]    = {0};
float temperature_readings[max_readings] = {0};
float humidity_readings[max_readings]    = {0};
float rain_readings[max_readings]        = {0};
float snow_readings[max_readings]        = {0};

long SleepDuration = 60; // Sleep time in minutes, aligned to the nearest minute boundary, so if 30 will always update at 00 or 30 past the hour
int  WakeupHour    = 8;  // Don't wakeup until after 07:00 to save battery power
int  SleepHour     = 23; // Sleep after 23:00 to save battery power
long StartTime     = 0;
long SleepTimer    = 0;
long Delta         = 30; // ESP32 rtc speed compensation, prevents display at xx:59:yy and then xx:00:yy (one minute later) to save power

//fonts
#include "opensans8b.h"
#include "opensans10b.h"
#include "opensans12b.h"
#include "opensans18b.h"
#include "opensans24b.h"

GFXfont currentFont;
uint8_t *framebuffer;

void BeginSleep()
{
    epd_poweroff_all();
    UpdateLocalTime();
    SleepTimer = (SleepDuration * 60 - ((CurrentMin % SleepDuration) * 60 + CurrentSec)) + Delta; //Some ESP32 have a RTC that is too fast to maintain accurate time, so add an offset
    esp_sleep_enable_timer_wakeup(SleepTimer * 1000000LL);                                        // in Secs, 1000000LL converts to Secs as unit = 1uSec
    Serial.println("Awake for : " + String((millis() - StartTime) / 1000.0, 3) + "-secs");
    Serial.println("Entering " + String(SleepTimer) + " (secs) of sleep time");
    Serial.println("Starting deep-sleep period...");
    esp_deep_sleep_start(); // Sleep for e.g. 30 minutes
}

boolean SetupTime()
{
    configTime(gmtOffset_sec, daylightOffset_sec, const_cast<const char *>(ntpServer.c_str()), "time.nist.gov"); //(gmtOffset_sec, daylightOffset_sec, ntpServer)
    setenv("TZ", const_cast<const char *>(Timezone.c_str()), 1);                                                 //setenv()adds the "TZ" variable to the environment with a value TimeZone, only used if set to 1, 0 means no change
    tzset();                                                                   // Set the TZ environment variable
    delay(100);
    return UpdateLocalTime();
}

uint8_t StartWiFi()
{
    Serial.println("\r\nConnecting to: " + String(ssid));
    IPAddress dns(8, 8, 8, 8); // Use Google DNS
    WiFi.disconnect();
    WiFi.mode(WIFI_STA); // switch off AP
    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);
    WiFi.begin(const_cast<const char *>(ssid.c_str()), const_cast<const char *>(password.c_str()));
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.printf("STA: Failed!\n");
        WiFi.disconnect(false);
        delay(500);
        WiFi.begin(const_cast<const char *>(ssid.c_str()), const_cast<const char *>(password.c_str()));
    }
    if (WiFi.status() == WL_CONNECTED)
    {
        wifi_signal = WiFi.RSSI(); // Get Wifi Signal strength now, because the WiFi will be turned off to save power!
        Serial.println("WiFi connected at: " + WiFi.localIP().toString());
    }
    else
        Serial.println("WiFi connection *** FAILED ***");
    return WiFi.status();
}

void StopWiFi()
{
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi switched Off");
}

void InitialiseSystem()
{
    StartTime = millis();
    Serial.begin(115200);
    while (!Serial) ;
    Serial.println(String(__FILE__) + "\nStarting...");
    epd_init();
    framebuffer = (uint8_t *)ps_calloc(sizeof(uint8_t), EPD_WIDTH * EPD_HEIGHT / 2);
    if (!framebuffer)
        Serial.println("Memory alloc failed!");
    memset(framebuffer, 0xFF, EPD_WIDTH * EPD_HEIGHT / 2);
}

void loop()
{
    // Nothing to do here
    delay(1000);
}

#if T5_47_PLUS_V2
#define USR_BUTTON 21
#elif CONFIG_IDF_TARGET_ESP32
#define USR_BUTTON 35
#elif CONFIG_IDF_TARGET_ESP32S3
#define USR_BUTTON 21
#endif

void setup()
{
    InitialiseSystem();

    SPIFFS.begin();
    pinMode(USR_BUTTON, INPUT_PULLUP);

    if ( !digitalRead(USR_BUTTON) || !SPIFFS.exists("/config.json") )
    {
        setupWEB();
    }
    else
    {
        StaticJsonDocument<512> json1;
        File configfile = SPIFFS.open("/config.json", "r");
        DeserializationError error = deserializeJson(json1, configfile);
        if (error)
            Serial.println(F("Failed to read file, using default configuration"));
        configfile.close();

        ssid = json1["WLAN"]["ssid"].as<String>();
        password = json1["WLAN"]["password"].as<String>();

        apikey     = json1["OpenWeather"]["apikey"].as<String>();
        server     = json1["OpenWeather"]["server"].as<String>();
        Country    = json1["OpenWeather"]["country"].as<String>();
        City       = json1["OpenWeather"]["city"].as<String>();
        Hemisphere = json1["OpenWeather"]["hemisphere"].as<String>();
        Units      = json1["OpenWeather"]["units"].as<String>();

        ntpServer = json1["ntp"]["server"].as<String>();
        Timezone = json1["ntp"]["timezone"].as<String>();

        // WakeupHour = strtok(json1["schedule_power"]["on_time"].as<const char *>());
        // SleepHour  = json1["schedule_power"]["off_time"].as<String>();

        if (StartWiFi() == WL_CONNECTED && SetupTime() == true)
        {
            // bool WakeUp = false;
            // if (WakeupHour > SleepHour)
            //     WakeUp = (CurrentHour >= WakeupHour || CurrentHour <= SleepHour);
            // else
            //     WakeUp = (CurrentHour >= WakeupHour && CurrentHour <= SleepHour);
            // if (WakeUp)
            // {
                byte Attempts = 1;
                bool RxWeather = false;
                bool RxForecast = false;
                WiFiClient client; // wifi client object
                while ((RxWeather == false || RxForecast == false) && Attempts <= 2)
                { // Try up-to 2 time for Weather and Forecast data
                    if (RxWeather == false)
                        RxWeather = obtainWeatherData(client, "weather");
                    if (RxForecast == false)
                        RxForecast = obtainWeatherData(client, "forecast");
                    Attempts++;
                }
                Serial.println("Received all weather data...");
                if (RxWeather && RxForecast)
                {                       // Only if received both Weather or Forecast proceed
                    StopWiFi();         // Reduces power consumption
                    epd_poweron();      // Switch on EPD display
                    epd_clear();        // Clear the screen
                    DisplayWeather();   // Display the weather data
                    edp_update();       // Update the display to show the information
                    epd_poweroff_all(); // Switch off all power to EPD
                }
            // }
        }
        BeginSleep();
    }
}

void Convert_Readings_to_Imperial()
{ // Only the first 3-hours are used
    WxConditions[0].Pressure = hPa_to_inHg(WxConditions[0].Pressure);
    WxForecast[0].Rainfall = mm_to_inches(WxForecast[0].Rainfall);
    WxForecast[0].Snowfall = mm_to_inches(WxForecast[0].Snowfall);
}

bool DecodeWeather(WiFiClient &json, String Type)
{
    Serial.print(F("\nCreating object...and "));
    DynamicJsonDocument doc(64 * 1024);                      // allocate the JsonDocument
    DeserializationError error = deserializeJson(doc, json); // Deserialize the JSON document
    if (error)
    { // Test if parsing succeeds.
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.c_str());
        return false;
    }
    // convert it to a JsonObject
    JsonObject root = doc.as<JsonObject>();
    Serial.println(" Decoding " + Type + " data");
    if (Type == "weather")
    {
        // All Serial.println statements are for diagnostic purposes and some are not required, remove if not needed with //
        //WxConditions[0].lon         = root["coord"]["lon"].as<float>();              Serial.println(" Lon: " + String(WxConditions[0].lon));
        //WxConditions[0].lat         = root["coord"]["lat"].as<float>();              Serial.println(" Lat: " + String(WxConditions[0].lat));
        WxConditions[0].Main0 = root["weather"][0]["main"].as<const char *>();
        Serial.println("Main: " + String(WxConditions[0].Main0));
        WxConditions[0].Forecast0 = root["weather"][0]["description"].as<const char *>();
        Serial.println("For0: " + String(WxConditions[0].Forecast0));
        //WxConditions[0].Forecast1   = root["weather"][1]["description"].as<char*>(); Serial.println("For1: " + String(WxConditions[0].Forecast1));
        //WxConditions[0].Forecast2   = root["weather"][2]["description"].as<char*>(); Serial.println("For2: " + String(WxConditions[0].Forecast2));
        WxConditions[0].Icon = root["weather"][0]["icon"].as<const char *>();
        Serial.println("Icon: " + String(WxConditions[0].Icon));
        WxConditions[0].Temperature = root["main"]["temp"].as<float>();
        Serial.println("Temp: " + String(WxConditions[0].Temperature));
        WxConditions[0].Pressure = root["main"]["pressure"].as<float>();
        Serial.println("Pres: " + String(WxConditions[0].Pressure));
        WxConditions[0].Humidity = root["main"]["humidity"].as<float>();
        Serial.println("Humi: " + String(WxConditions[0].Humidity));
        WxConditions[0].Low = root["main"]["temp_min"].as<float>();
        Serial.println("TLow: " + String(WxConditions[0].Low));
        WxConditions[0].High = root["main"]["temp_max"].as<float>();
        Serial.println("THig: " + String(WxConditions[0].High));
        WxConditions[0].Windspeed = root["wind"]["speed"].as<float>();
        Serial.println("WSpd: " + String(WxConditions[0].Windspeed));
        WxConditions[0].Winddir = root["wind"]["deg"].as<float>();
        Serial.println("WDir: " + String(WxConditions[0].Winddir));
        WxConditions[0].Cloudcover = root["clouds"]["all"].as<int>();
        Serial.println("CCov: " + String(WxConditions[0].Cloudcover)); // in % of cloud cover
        WxConditions[0].Visibility = root["visibility"].as<int>();
        Serial.println("Visi: " + String(WxConditions[0].Visibility)); // in metres
        WxConditions[0].Rainfall = root["rain"]["1h"].as<float>();
        Serial.println("Rain: " + String(WxConditions[0].Rainfall));
        WxConditions[0].Snowfall = root["snow"]["1h"].as<float>();
        Serial.println("Snow: " + String(WxConditions[0].Snowfall));
        //WxConditions[0].Country     = root["sys"]["country"].as<char*>();            Serial.println("Ctry: " + String(WxConditions[0].Country));
        WxConditions[0].Sunrise = root["sys"]["sunrise"].as<int>();
        Serial.println("SRis: " + String(WxConditions[0].Sunrise));
        WxConditions[0].Sunset = root["sys"]["sunset"].as<int>();
        Serial.println("SSet: " + String(WxConditions[0].Sunset));
        WxConditions[0].Timezone = root["timezone"].as<int>();
        Serial.println("TZon: " + String(WxConditions[0].Timezone));
    }
    if (Type == "forecast")
    {
        //Serial.println(json);
        Serial.print(F("\nReceiving Forecast period - ")); //------------------------------------------------
        JsonArray list = root["list"];
        for (byte r = 0; r < max_readings; r++)
        {
            Serial.println("\nPeriod-" + String(r) + "--------------");
            WxForecast[r].Dt = list[r]["dt"].as<int>();
            WxForecast[r].Temperature = list[r]["main"]["temp"].as<float>();
            Serial.println("Temp: " + String(WxForecast[r].Temperature));
            WxForecast[r].Low = list[r]["main"]["temp_min"].as<float>();
            Serial.println("TLow: " + String(WxForecast[r].Low));
            WxForecast[r].High = list[r]["main"]["temp_max"].as<float>();
            Serial.println("THig: " + String(WxForecast[r].High));
            WxForecast[r].Pressure = list[r]["main"]["pressure"].as<float>();
            Serial.println("Pres: " + String(WxForecast[r].Pressure));
            WxForecast[r].Humidity = list[r]["main"]["humidity"].as<float>();
            Serial.println("Humi: " + String(WxForecast[r].Humidity));
            //WxForecast[r].Forecast0         = list[r]["weather"][0]["main"].as<char*>();        Serial.println("For0: " + String(WxForecast[r].Forecast0));
            //WxForecast[r].Forecast1         = list[r]["weather"][1]["main"].as<char*>();        Serial.println("For1: " + String(WxForecast[r].Forecast1));
            //WxForecast[r].Forecast2         = list[r]["weather"][2]["main"].as<char*>();        Serial.println("For2: " + String(WxForecast[r].Forecast2));
            WxForecast[r].Icon = list[r]["weather"][0]["icon"].as<const char *>();
            Serial.println("Icon: " + String(WxForecast[r].Icon));
            //WxForecast[r].Description       = list[r]["weather"][0]["description"].as<char*>(); Serial.println("Desc: " + String(WxForecast[r].Description));
            //WxForecast[r].Cloudcover        = list[r]["clouds"]["all"].as<int>();               Serial.println("CCov: " + String(WxForecast[r].Cloudcover)); // in % of cloud cover
            //WxForecast[r].Windspeed         = list[r]["wind"]["speed"].as<float>();             Serial.println("WSpd: " + String(WxForecast[r].Windspeed));
            //WxForecast[r].Winddir           = list[r]["wind"]["deg"].as<float>();               Serial.println("WDir: " + String(WxForecast[r].Winddir));
            WxForecast[r].Rainfall = list[r]["rain"]["3h"].as<float>();
            Serial.println("Rain: " + String(WxForecast[r].Rainfall));
            WxForecast[r].Snowfall = list[r]["snow"]["3h"].as<float>();
            Serial.println("Snow: " + String(WxForecast[r].Snowfall));
            WxForecast[r].Period = list[r]["dt_txt"].as<const char *>();
            Serial.println("Peri: " + String(WxForecast[r].Period));
        }
        //------------------------------------------
        float pressure_trend = WxForecast[0].Pressure - WxForecast[2].Pressure; // Measure pressure slope between ~now and later
        pressure_trend = ((int)(pressure_trend * 10)) / 10.0;                   // Remove any small variations less than 0.1
        WxConditions[0].Trend = "=";
        if (pressure_trend > 0)
            WxConditions[0].Trend = "+";
        if (pressure_trend < 0)
            WxConditions[0].Trend = "-";
        if (pressure_trend == 0)
            WxConditions[0].Trend = "0";

        if (Units == "I")
            Convert_Readings_to_Imperial();
    }
    return true;
}
//#########################################################################################
String ConvertUnixTime(int unix_time)
{
    // Returns either '21:12  ' or ' 09:12pm' depending on Units mode
    time_t tm = unix_time;
    struct tm *now_tm = localtime(&tm);
    char output[40];
    if (Units == "M")
    {
        strftime(output, sizeof(output), "%H:%M %d/%m/%y", now_tm);
    }
    else
    {
        strftime(output, sizeof(output), "%I:%M%P %m/%d/%y", now_tm);
    }
    return output;
}
//#########################################################################################
bool obtainWeatherData(WiFiClient &client, const String &RequestType)
{
    const String units = (Units == "M" ? "metric" : "imperial");
    client.stop(); // close connection before sending a new request
    HTTPClient http;
    String uri = "/data/2.5/" + RequestType + "?q=" + urlEncode(City) + "," + Country + "&APPID=" + apikey + "&mode=json&units=" + units + "&lang=" + Language;
    if (RequestType != "weather")
    {
        uri += "&cnt=" + String(max_readings);
    }
    http.begin(client, server, 80, uri); //http.begin(uri,test_root_ca); //HTTPS example connection
    int httpCode = http.GET();
    if (httpCode == HTTP_CODE_OK)
    {
        if (!DecodeWeather(http.getStream(), RequestType))
            return false;
        client.stop();
        http.end();
        return true;
    }
    else
    {
        Serial.printf("connection failed, error: %s", http.errorToString(httpCode).c_str());
        client.stop();
        http.end();
        return false;
    }
    http.end();
    return true;
}

float mm_to_inches(float value_mm)
{
    return 0.0393701 * value_mm;
}

float hPa_to_inHg(float value_hPa)
{
    return 0.02953 * value_hPa;
}

int JulianDate(int d, int m, int y)
{
    int mm, yy, k1, k2, k3, j;
    yy = y - (int)((12 - m) / 10);
    mm = m + 9;
    if (mm >= 12)
        mm = mm - 12;
    k1 = (int)(365.25 * (yy + 4712));
    k2 = (int)(30.6001 * mm + 0.5);
    k3 = (int)((int)((yy / 100) + 49) * 0.75) - 38;
    // 'j' for dates in Julian calendar:
    j = k1 + k2 + d + 59 + 1;
    if (j > 2299160)
        j = j - k3; // 'j' is the Julian date at 12h UT (Universal Time) For Gregorian calendar:
    return j;
}

float SumOfPrecip(float DataArray[], int readings)
{
    float sum = 0;
    for (int i = 0; i <= readings; i++)
    {
        sum += DataArray[i];
    }
    return sum;
}

String TitleCase(String text)
{
    if (text.length() > 0)
    {
        String temp_text = text.substring(0, 1);
        temp_text.toUpperCase();
        return temp_text + text.substring(1); // Title-case the string
    }
    else
        return text;
}

double NormalizedMoonPhase(int d, int m, int y)
{
    int j = JulianDate(d, m, y);
    //Calculate approximate moon phase
    double Phase = (j + 4.867) / 29.53059;
    return (Phase - (int)Phase);
}

void DisplayWeather()
{                                               // 4.7" e-paper display is 960x540 resolution
    DisplayStatusSection(600, 20, wifi_signal); // Wi-Fi signal strength and Battery voltage
    DisplayGeneralInfoSection();                // Top line of the display
    DisplayDisplayWindSection(137, 150, WxConditions[0].Winddir, WxConditions[0].Windspeed, 100);
    DisplayAstronomySection(5, 255);     // Astronomy section Sun rise/set, Moon phase and Moon icon
    DisplayMainWeatherSection(320, 110); // Centre section of display for Location, temperature, Weather report, current Wx Symbol
    DisplayWeatherIcon(810, 130);        // Display weather icon    scale = Large;
    DisplayForecastSection(320, 220);    // 3hr forecast boxes
}

void DisplayGeneralInfoSection()
{
    setFont(OpenSans10B);
    drawString(5, 2, City, LEFT);
    setFont(OpenSans8B);
    drawString(500, 2, Date_str + "  @   " + Time_str, LEFT);
}

void DisplayWeatherIcon(int x, int y)
{
    DisplayConditionsSection(x, y, WxConditions[0].Icon, LargeIcon);
}

void DisplayMainWeatherSection(int x, int y)
{
    setFont(OpenSans8B);
    DisplayTemperatureSection(x, y - 40);
    DisplayForecastTextSection(x - 55, y + 25);
    DisplayPressureSection(x - 25, y + 90, WxConditions[0].Pressure, WxConditions[0].Trend);
}

void DisplayDisplayWindSection(int x, int y, float angle, float windspeed, int Cradius)
{
    arrow(x, y, Cradius - 22, angle, 18, 33); // Show wind direction on outer circle of width and length
    setFont(OpenSans8B);
    int dxo, dyo, dxi, dyi;
    drawCircle(x, y, Cradius, Black);       // Draw compass circle
    drawCircle(x, y, Cradius + 1, Black);   // Draw compass circle
    drawCircle(x, y, Cradius * 0.7, Black); // Draw compass inner circle
    for (float a = 0; a < 360; a = a + 22.5)
    {
        dxo = Cradius * cos((a - 90) * PI / 180);
        dyo = Cradius * sin((a - 90) * PI / 180);
        if (a == 45)
            drawString(dxo + x + 15, dyo + y - 18, TXT_NE, CENTER);
        if (a == 135)
            drawString(dxo + x + 20, dyo + y - 2, TXT_SE, CENTER);
        if (a == 225)
            drawString(dxo + x - 20, dyo + y - 2, TXT_SW, CENTER);
        if (a == 315)
            drawString(dxo + x - 15, dyo + y - 18, TXT_NW, CENTER);
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
        dxo = dxo * 0.7;
        dyo = dyo * 0.7;
        dxi = dxo * 0.9;
        dyi = dyo * 0.9;
        drawLine(dxo + x, dyo + y, dxi + x, dyi + y, Black);
    }
    drawString(x, y - Cradius - 20, TXT_N, CENTER);
    drawString(x, y + Cradius + 10, TXT_S, CENTER);
    drawString(x - Cradius - 15, y - 5, TXT_W, CENTER);
    drawString(x + Cradius + 10, y - 5, TXT_E, CENTER);
    drawString(x + 3, y + 50, String(angle, 0) + "°", CENTER);
    setFont(OpenSans12B);
    drawString(x, y - 50, WindDegToOrdinalDirection(angle), CENTER);
    setFont(OpenSans24B);
    drawString(x + 3, y - 18, String(windspeed, 1), CENTER);
    setFont(OpenSans12B);
    drawString(x, y + 25, (Units == "M" ? "m/s" : "mph"), CENTER);
}

String WindDegToOrdinalDirection(float winddirection)
{
    if (winddirection >= 348.75 || winddirection < 11.25)
        return TXT_N;
    if (winddirection >= 11.25 && winddirection < 33.75)
        return TXT_NNE;
    if (winddirection >= 33.75 && winddirection < 56.25)
        return TXT_NE;
    if (winddirection >= 56.25 && winddirection < 78.75)
        return TXT_ENE;
    if (winddirection >= 78.75 && winddirection < 101.25)
        return TXT_E;
    if (winddirection >= 101.25 && winddirection < 123.75)
        return TXT_ESE;
    if (winddirection >= 123.75 && winddirection < 146.25)
        return TXT_SE;
    if (winddirection >= 146.25 && winddirection < 168.75)
        return TXT_SSE;
    if (winddirection >= 168.75 && winddirection < 191.25)
        return TXT_S;
    if (winddirection >= 191.25 && winddirection < 213.75)
        return TXT_SSW;
    if (winddirection >= 213.75 && winddirection < 236.25)
        return TXT_SW;
    if (winddirection >= 236.25 && winddirection < 258.75)
        return TXT_WSW;
    if (winddirection >= 258.75 && winddirection < 281.25)
        return TXT_W;
    if (winddirection >= 281.25 && winddirection < 303.75)
        return TXT_WNW;
    if (winddirection >= 303.75 && winddirection < 326.25)
        return TXT_NW;
    if (winddirection >= 326.25 && winddirection < 348.75)
        return TXT_NNW;
    return "?";
}

void DisplayTemperatureSection(int x, int y)
{
    setFont(OpenSans18B);
    drawString(x - 30, y, String(WxConditions[0].Temperature, 1) + "°    " + String(WxConditions[0].Humidity, 0) + "%", LEFT);
    setFont(OpenSans12B);
    drawString(x + 10, y + 35, String(WxConditions[0].High, 0) + "° | " + String(WxConditions[0].Low, 0) + "°", CENTER); // Show forecast high and Low
}

void DisplayForecastTextSection(int x, int y)
{
#define lineWidth 34
    setFont(OpenSans12B);
    //Wx_Description = WxConditions[0].Main0;          // e.g. typically 'Clouds'
    String Wx_Description = WxConditions[0].Forecast0; // e.g. typically 'overcast clouds' ... you choose which
    Wx_Description.replace(".", "");                   // remove any '.'
    int spaceRemaining = 0, p = 0, charCount = 0, Width = lineWidth;
    while (p < Wx_Description.length())
    {
        if (Wx_Description.substring(p, p + 1) == " ")
            spaceRemaining = p;
        if (charCount > Width - 1)
        { // '~' is the end of line marker
            Wx_Description = Wx_Description.substring(0, spaceRemaining) + "~" + Wx_Description.substring(spaceRemaining + 1);
            charCount = 0;
        }
        p++;
        charCount++;
    }
    if (WxForecast[0].Rainfall > 0)
        Wx_Description += " (" + String(WxForecast[0].Rainfall, 1) + String((Units == "M" ? "mm" : "in")) + ")";
    //Wx_Description = wordWrap(Wx_Description, lineWidth);
    String Line1 = Wx_Description.substring(0, Wx_Description.indexOf("~"));
    String Line2 = Wx_Description.substring(Wx_Description.indexOf("~") + 1);
    drawString(x + 30, y + 5, TitleCase(Line1), LEFT);
    if (Line1 != Line2)
        drawString(x + 30, y + 30, Line2, LEFT);
}

void DisplayPressureSection(int x, int y, float pressure, String slope)
{
    setFont(OpenSans12B);
    DrawPressureAndTrend(x - 25, y + 10, pressure, slope);
    if (WxConditions[0].Visibility > 0)
    {
        Visibility(x + 145, y, String(WxConditions[0].Visibility) + "M");
        x += 150; // Draw the text in the same positions if one is zero, otherwise in-line
    }
    if (WxConditions[0].Cloudcover > 0)
        CloudCover(x + 145, y, WxConditions[0].Cloudcover);
}

void DisplayForecastWeather(int x, int y, int index)
{
    int fwidth = 90;
    x = x + fwidth * index;
    DisplayConditionsSection(x + fwidth / 2, y + 90, WxForecast[index].Icon, SmallIcon);
    setFont(OpenSans10B);
    drawString(x + fwidth / 2, y + 30, String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone).substring(0, 5)), CENTER);
    drawString(x + fwidth / 2, y + 125, String(WxForecast[index].High, 0) + "°/" + String(WxForecast[index].Low, 0) + "°", CENTER);
}

void DisplayAstronomySection(int x, int y)
{
    setFont(OpenSans10B);
    drawString(x + 5, y + 30, ConvertUnixTime(WxConditions[0].Sunrise).substring(0, 5) + " " + TXT_SUNRISE, LEFT);
    drawString(x + 5, y + 50, ConvertUnixTime(WxConditions[0].Sunset).substring(0, 5) + " " + TXT_SUNSET, LEFT);
    time_t now = time(NULL);
    struct tm *now_utc = gmtime(&now);
    const int day_utc = now_utc->tm_mday;
    const int month_utc = now_utc->tm_mon + 1;
    const int year_utc = now_utc->tm_year + 1900;
    drawString(x + 5, y + 70, MoonPhase(day_utc, month_utc, year_utc, Hemisphere), LEFT);
    DrawMoon(x + 160, y - 15, day_utc, month_utc, year_utc, Hemisphere);
}

void DrawMoon(int x, int y, int dd, int mm, int yy, String hemisphere)
{
    const int diameter = 75;
    double Phase = NormalizedMoonPhase(dd, mm, yy);
    hemisphere.toLowerCase();
    if (hemisphere == "south")
        Phase = 1 - Phase;
    // Draw dark part of moon
    fillCircle(x + diameter - 1, y + diameter, diameter / 2 + 1, LightGrey);
    const int number_of_lines = 90;
    for (double Ypos = 0; Ypos <= number_of_lines / 2; Ypos++)
    {
        double Xpos = sqrt(number_of_lines / 2 * number_of_lines / 2 - Ypos * Ypos);
        // Determine the edges of the lighted part of the moon
        double Rpos = 2 * Xpos;
        double Xpos1, Xpos2;
        if (Phase < 0.5)
        {
            Xpos1 = -Xpos;
            Xpos2 = Rpos - 2 * Phase * Rpos - Xpos;
        }
        else
        {
            Xpos1 = Xpos;
            Xpos2 = Xpos - 2 * Phase * Rpos + Rpos;
        }
        // Draw light part of moon
        double pW1x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW1y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW2x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW2y = (number_of_lines - Ypos) / number_of_lines * diameter + y;
        double pW3x = (Xpos1 + number_of_lines) / number_of_lines * diameter + x;
        double pW3y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        double pW4x = (Xpos2 + number_of_lines) / number_of_lines * diameter + x;
        double pW4y = (Ypos + number_of_lines) / number_of_lines * diameter + y;
        drawLine(pW1x, pW1y, pW2x, pW2y, White);
        drawLine(pW3x, pW3y, pW4x, pW4y, White);
    }
    drawCircle(x + diameter - 1, y + diameter, diameter / 2, Black);
}

String MoonPhase(int d, int m, int y, String hemisphere)
{
    int c, e;
    double jd;
    int b;
    if (m < 3)
    {
        y--;
        m += 12;
    }
    ++m;
    c = 365.25 * y;
    e = 30.6 * m;
    jd = c + e + d - 694039.09; /* jd is total days elapsed */
    jd /= 29.53059;             /* divide by the moon cycle (29.53 days) */
    b = jd;                     /* int(jd) -> b, take integer part of jd */
    jd -= b;                    /* subtract integer part to leave fractional part of original jd */
    b = jd * 8 + 0.5;           /* scale fraction from 0-8 and round by adding 0.5 */
    b = b & 7;                  /* 0 and 8 are the same phase so modulo 8 for 0 */
    if (hemisphere == "south")
        b = 7 - b;
    if (b == 0)
        return TXT_MOON_NEW; // New;              0%  illuminated
    if (b == 1)
        return TXT_MOON_WAXING_CRESCENT; // Waxing crescent; 25%  illuminated
    if (b == 2)
        return TXT_MOON_FIRST_QUARTER; // First quarter;   50%  illuminated
    if (b == 3)
        return TXT_MOON_WAXING_GIBBOUS; // Waxing gibbous;  75%  illuminated
    if (b == 4)
        return TXT_MOON_FULL; // Full;            100% illuminated
    if (b == 5)
        return TXT_MOON_WANING_GIBBOUS; // Waning gibbous;  75%  illuminated
    if (b == 6)
        return TXT_MOON_THIRD_QUARTER; // Third quarter;   50%  illuminated
    if (b == 7)
        return TXT_MOON_WANING_CRESCENT; // Waning crescent; 25%  illuminated
    return "";
}

void DisplayForecastSection(int x, int y)
{
    int f = 0;
    do
    {
        DisplayForecastWeather(x, y, f);
        f++;
    } while (f < max_readings);
    int r = 0;
    do
    { // Pre-load temporary arrays with with data - because C parses by reference and remember that[1] has already been converted to I units
        if (Units == "I")
            pressure_readings[r] = WxForecast[r].Pressure * 0.02953;
        else
            pressure_readings[r] = WxForecast[r].Pressure;
        if (Units == "I")
            rain_readings[r] = WxForecast[r].Rainfall * 0.0393701;
        else
            rain_readings[r] = WxForecast[r].Rainfall;
        if (Units == "I")
            snow_readings[r] = WxForecast[r].Snowfall * 0.0393701;
        else
            snow_readings[r] = WxForecast[r].Snowfall;
        temperature_readings[r] = WxForecast[r].Temperature;
        humidity_readings[r] = WxForecast[r].Humidity;
        r++;
    } while (r < max_readings);
    int gwidth = 175, gheight = 100;
    int gx = (SCREEN_WIDTH - gwidth * 4) / 5 + 8;
    int gy = (SCREEN_HEIGHT - gheight - 30);
    int gap = gwidth + gx;
    // (x,y,width,height,MinValue, MaxValue, Title, Data Array, AutoScale, ChartMode)
    DrawGraph(gx + 0 * gap, gy, gwidth, gheight, 900, 1050, Units == "M" ? TXT_PRESSURE_HPA : TXT_PRESSURE_IN, pressure_readings, max_readings, autoscale_on, barchart_off);
    DrawGraph(gx + 1 * gap, gy, gwidth, gheight, 10, 30, Units == "M" ? TXT_TEMPERATURE_C : TXT_TEMPERATURE_F, temperature_readings, max_readings, autoscale_on, barchart_off);
    DrawGraph(gx + 2 * gap, gy, gwidth, gheight, 0, 100, TXT_HUMIDITY_PERCENT, humidity_readings, max_readings, autoscale_off, barchart_off);
    if (SumOfPrecip(rain_readings, max_readings) >= SumOfPrecip(snow_readings, max_readings))
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_RAINFALL_MM : TXT_RAINFALL_IN, rain_readings, max_readings, autoscale_on, barchart_on);
    else
        DrawGraph(gx + 3 * gap + 5, gy, gwidth, gheight, 0, 30, Units == "M" ? TXT_SNOWFALL_MM : TXT_SNOWFALL_IN, snow_readings, max_readings, autoscale_on, barchart_on);
}

void DisplayConditionsSection(int x, int y, String IconName, bool IconSize)
{
    Serial.println("Icon name: " + IconName);
    if (IconName == "01d" || IconName == "01n")
        Sunny(x, y, IconSize, IconName);
    else if (IconName == "02d" || IconName == "02n")
        MostlySunny(x, y, IconSize, IconName);
    else if (IconName == "03d" || IconName == "03n")
        Cloudy(x, y, IconSize, IconName);
    else if (IconName == "04d" || IconName == "04n")
        MostlySunny(x, y, IconSize, IconName);
    else if (IconName == "09d" || IconName == "09n")
        ChanceRain(x, y, IconSize, IconName);
    else if (IconName == "10d" || IconName == "10n")
        Rain(x, y, IconSize, IconName);
    else if (IconName == "11d" || IconName == "11n")
        Tstorms(x, y, IconSize, IconName);
    else if (IconName == "13d" || IconName == "13n")
        Snow(x, y, IconSize, IconName);
    else if (IconName == "50d")
        Haze(x, y, IconSize, IconName);
    else if (IconName == "50n")
        Fog(x, y, IconSize, IconName);
    else
        Nodata(x, y, IconSize, IconName);
}

void arrow(int x, int y, int asize, float aangle, int pwidth, int plength)
{
    float dx = (asize - 10) * cos((aangle - 90) * PI / 180) + x; // calculate X position
    float dy = (asize - 10) * sin((aangle - 90) * PI / 180) + y; // calculate Y position
    float x1 = 0;
    float y1 = plength;
    float x2 = pwidth / 2;
    float y2 = pwidth / 2;
    float x3 = -pwidth / 2;
    float y3 = pwidth / 2;
    float angle = aangle * PI / 180 - 135;
    float xx1 = x1 * cos(angle) - y1 * sin(angle) + dx;
    float yy1 = y1 * cos(angle) + x1 * sin(angle) + dy;
    float xx2 = x2 * cos(angle) - y2 * sin(angle) + dx;
    float yy2 = y2 * cos(angle) + x2 * sin(angle) + dy;
    float xx3 = x3 * cos(angle) - y3 * sin(angle) + dx;
    float yy3 = y3 * cos(angle) + x3 * sin(angle) + dy;
    fillTriangle(xx1, yy1, xx3, yy3, xx2, yy2, Black);
}

void DrawSegment(int x, int y, int o1, int o2, int o3, int o4, int o11, int o12, int o13, int o14)
{
    drawLine(x + o1, y + o2, x + o3, y + o4, Black);
    drawLine(x + o11, y + o12, x + o13, y + o14, Black);
}

void DrawPressureAndTrend(int x, int y, float pressure, String slope)
{
    drawString(x + 25, y - 10, String(pressure, (Units == "M" ? 0 : 1)) + (Units == "M" ? "hPa" : "in"), LEFT);
    if (slope == "+")
    {
        DrawSegment(x, y, 0, 0, 8, -8, 8, -8, 16, 0);
        DrawSegment(x - 1, y, 0, 0, 8, -8, 8, -8, 16, 0);
    }
    else if (slope == "0")
    {
        DrawSegment(x, y, 8, -8, 16, 0, 8, 8, 16, 0);
        DrawSegment(x - 1, y, 8, -8, 16, 0, 8, 8, 16, 0);
    }
    else if (slope == "-")
    {
        DrawSegment(x, y, 0, 0, 8, 8, 8, 8, 16, 0);
        DrawSegment(x - 1, y, 0, 0, 8, 8, 8, 8, 16, 0);
    }
}

void DisplayStatusSection(int x, int y, int rssi)
{
    setFont(OpenSans8B);
    DrawRSSI(x + 305, y + 15, rssi);
    DrawBattery(x + 150, y);
}

void DrawRSSI(int x, int y, int rssi)
{
    int WIFIsignal = 0;
    int xpos = 1;
    for (int _rssi = -100; _rssi <= rssi; _rssi = _rssi + 20)
    {
        if (_rssi <= -20)
            WIFIsignal = 30; //            <-20dbm displays 5-bars
        if (_rssi <= -40)
            WIFIsignal = 24; //  -40dbm to  -21dbm displays 4-bars
        if (_rssi <= -60)
            WIFIsignal = 18; //  -60dbm to  -41dbm displays 3-bars
        if (_rssi <= -80)
            WIFIsignal = 12; //  -80dbm to  -61dbm displays 2-bars
        if (_rssi <= -100)
            WIFIsignal = 6; // -100dbm to  -81dbm displays 1-bar
        fillRect(x + xpos * 8, y - WIFIsignal, 6, WIFIsignal, Black);
        xpos++;
    }
}

boolean UpdateLocalTime()
{
    struct tm timeinfo;
    char time_output[30], day_output[30], update_time[30];
    while (!getLocalTime(&timeinfo, 5000))
    { // Wait for 5-sec for time to synchronise
        Serial.println("Failed to obtain time");
        return false;
    }
    CurrentHour = timeinfo.tm_hour;
    CurrentMin = timeinfo.tm_min;
    CurrentSec = timeinfo.tm_sec;
    //See http://www.cplusplus.com/reference/ctime/strftime/
    Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday, June 24 2017 14:05:49
    if (Units == "M")
    {
        sprintf(day_output, "%s, %02u %s %04u", weekday_D[timeinfo.tm_wday], timeinfo.tm_mday, month_M[timeinfo.tm_mon], (timeinfo.tm_year) + 1900);
        strftime(update_time, sizeof(update_time), "%H:%M:%S", &timeinfo); // Creates: '@ 14:05:49'   and change from 30 to 8 <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        sprintf(time_output, "%s", update_time);
    }
    else
    {
        strftime(day_output, sizeof(day_output), "%a %b-%d-%Y", &timeinfo); // Creates  'Sat May-31-2019'
        strftime(update_time, sizeof(update_time), "%r", &timeinfo);        // Creates: '@ 02:05:49pm'
        sprintf(time_output, "%s", update_time);
    }
    Date_str = day_output;
    Time_str = time_output;
    return true;
}

void DrawBattery(int x, int y)
{
    uint8_t percentage = 100;
    esp_adc_cal_characteristics_t adc_chars;
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        Serial.printf("eFuse Vref:%u mV", adc_chars.vref);
        vref = adc_chars.vref;
    }
#if CONFIG_IDF_TARGET_ESP32
    const uint8_t bat_adc_pin = 36;
#else
    const uint8_t bat_adc_pin = 14;
#endif
    float voltage = analogRead(bat_adc_pin) / 4096.0 * 6.566 * (vref / 1000.0);
    if (voltage > 1)
    { // Only display if there is a valid reading
        Serial.println("\nVoltage = " + String(voltage));
        percentage = 2836.9625 * pow(voltage, 4) - 43987.4889 * pow(voltage, 3) + 255233.8134 * pow(voltage, 2) - 656689.7123 * voltage + 632041.7303;
        if (voltage >= 4.20)
            percentage = 100;
        if (voltage <= 3.20)
            percentage = 0; // orig 3.5
        drawRect(x + 25, y - 14, 40, 15, Black);
        fillRect(x + 65, y - 10, 4, 7, Black);
        fillRect(x + 27, y - 12, 36 * percentage / 100.0, 11, Black);
        drawString(x + 85, y - 14, String(percentage) + "%  " + String(voltage, 1) + "v", LEFT);
    }
}

// Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing unit
void addcloud(int x, int y, int scale, int linesize)
{
    fillCircle(x - scale * 3, y, scale, Black);                                                              // Left most circle
    fillCircle(x + scale * 3, y, scale, Black);                                                              // Right most circle
    fillCircle(x - scale, y - scale, scale * 1.4, Black);                                                    // left middle upper circle
    fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75, Black);                                       // Right middle upper circle
    fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1, Black);                                 // Upper and lower lines
    fillCircle(x - scale * 3, y, scale - linesize, White);                                                   // Clear left most circle
    fillCircle(x + scale * 3, y, scale - linesize, White);                                                   // Clear right most circle
    fillCircle(x - scale, y - scale, scale * 1.4 - linesize, White);                                         // left middle upper circle
    fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize, White);                            // Right middle upper circle
    fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9, scale * 2 - linesize * 2 + 2, White); // Upper and lower lines
}

void addrain(int x, int y, int scale, bool IconSize)
{
    if (IconSize == SmallIcon)
    {
        setFont(OpenSans8B);
        drawString(x - 25, y + 12, "///////", LEFT);
    }
    else
    {
        setFont(OpenSans18B);
        drawString(x - 60, y + 25, "///////", LEFT);
    }
}

void addsnow(int x, int y, int scale, bool IconSize)
{
    if (IconSize == SmallIcon)
    {
        setFont(OpenSans8B);
        drawString(x - 25, y + 15, "* * * *", LEFT);
    }
    else
    {
        setFont(OpenSans18B);
        drawString(x - 60, y + 30, "* * * *", LEFT);
    }
}

void addtstorm(int x, int y, int scale)
{
    y = y + scale / 2;
    for (int i = 0; i < 5; i++)
    {
        drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 0, y + scale, Black);
        if (scale != Small)
        {
            drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 1, y + scale, Black);
            drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5, x - scale * 3.5 + scale * i * 1.5 + 2, y + scale, Black);
        }
        drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0, Black);
        if (scale != Small)
        {
            drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1, Black);
            drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2, Black);
        }
        drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5, Black);
        if (scale != Small)
        {
            drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5, Black);
            drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5, x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5, Black);
        }
    }
}

void addsun(int x, int y, int scale, bool IconSize)
{
    int linesize = 5;
    fillRect(x - scale * 2, y, scale * 4, linesize, Black);
    fillRect(x, y - scale * 2, linesize, scale * 4, Black);
    drawLine(x - scale * 1.3, y - scale * 1.3, x + scale * 1.3, y + scale * 1.3, Black);
    drawLine(x - scale * 1.3, y + scale * 1.3, x + scale * 1.3, y - scale * 1.3, Black);
    if (IconSize == LargeIcon)
    {
        drawLine(1 + x - scale * 1.3, y - scale * 1.3, 1 + x + scale * 1.3, y + scale * 1.3, Black);
        drawLine(2 + x - scale * 1.3, y - scale * 1.3, 2 + x + scale * 1.3, y + scale * 1.3, Black);
        drawLine(3 + x - scale * 1.3, y - scale * 1.3, 3 + x + scale * 1.3, y + scale * 1.3, Black);
        drawLine(1 + x - scale * 1.3, y + scale * 1.3, 1 + x + scale * 1.3, y - scale * 1.3, Black);
        drawLine(2 + x - scale * 1.3, y + scale * 1.3, 2 + x + scale * 1.3, y - scale * 1.3, Black);
        drawLine(3 + x - scale * 1.3, y + scale * 1.3, 3 + x + scale * 1.3, y - scale * 1.3, Black);
    }
    fillCircle(x, y, scale * 1.3, White);
    fillCircle(x, y, scale, Black);
    fillCircle(x, y, scale - linesize, White);
}

void addfog(int x, int y, int scale, int linesize, bool IconSize)
{
    if (IconSize == SmallIcon)
    {
        y -= 10;
        linesize = 1;
    }
    for (int i = 0; i < 6; i++)
    {
        fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize, Black);
        fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize, Black);
        fillRect(x - scale * 3, y + scale * 2.5, scale * 6, linesize, Black);
    }
}

void Sunny(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    else
        y = y - 3; // Shift up small sun icon
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    scale = scale * 1.6;
    addsun(x, y, scale, IconSize);
}

void MostlySunny(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
    addcloud(x, y, scale, linesize);
}

void MostlyCloudy(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
}

void Cloudy(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x + 15, y - 22, scale / 2, linesize); // Cloud top right
    addcloud(x - 10, y - 18, scale / 2, linesize); // Cloud top left
    addcloud(x, y, scale, linesize);               // Main cloud
}

void Rain(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addrain(x, y, scale, IconSize);
}

void ExpectRain(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addrain(x, y, scale, IconSize);
}

void ChanceRain(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    ;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addsun(x - scale * 1.8, y - scale * 1.8, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addrain(x, y, scale, IconSize);
}

void Tstorms(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addtstorm(x, y, scale);
}

void Snow(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x, y, scale, linesize);
    addsnow(x, y, scale, IconSize);
}

void Fog(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addcloud(x, y - 5, scale, linesize);
    addfog(x, y - 5, scale, linesize, IconSize);
}

void Haze(int x, int y, bool IconSize, String IconName)
{
    int scale = Small, linesize = 5, Offset = 10;
    if (IconSize == LargeIcon)
    {
        scale = Large;
        Offset = 35;
    }
    if (IconName.endsWith("n"))
        addmoon(x, y + Offset, scale, IconSize);
    addsun(x, y - 5, scale * 1.4, IconSize);
    addfog(x, y - 5, scale * 1.4, linesize, IconSize);
}

void CloudCover(int x, int y, int CCover)
{
    addcloud(x - 9, y + 2, Small * 0.3, 2); // Cloud top left
    addcloud(x + 3, y - 2, Small * 0.3, 2); // Cloud top right
    addcloud(x, y + 10, Small * 0.6, 2);    // Main cloud
    drawString(x + 20, y, String(CCover) + "%", LEFT);
}

void Visibility(int x, int y, String Visi)
{
    float start_angle = 0.52, end_angle = 2.61, Offset = 8;
    int r = 14;
    for (float i = start_angle; i < end_angle; i = i + 0.05)
    {
        drawPixel(x + r * cos(i), y - r / 2 + r * sin(i) + Offset, Black);
        drawPixel(x + r * cos(i), 1 + y - r / 2 + r * sin(i) + Offset, Black);
    }
    start_angle = 3.61;
    end_angle = 5.78;
    for (float i = start_angle; i < end_angle; i = i + 0.05)
    {
        drawPixel(x + r * cos(i), y + r / 2 + r * sin(i) + Offset, Black);
        drawPixel(x + r * cos(i), 1 + y + r / 2 + r * sin(i) + Offset, Black);
    }
    fillCircle(x, y + Offset, r / 4, Black);
    drawString(x + 20, y, Visi, LEFT);
}

void addmoon(int x, int y, int scale, bool IconSize)
{
    if (IconSize == LargeIcon)
    {
        fillCircle(x - 85, y - 100, uint16_t(scale * 0.8), Black);
        fillCircle(x - 57, y - 100, uint16_t(scale * 1.6), White);
    }
    else
    {
        fillCircle(x - 28, y - 37, uint16_t(scale * 1.0), Black);
        fillCircle(x - 20, y - 37, uint16_t(scale * 1.6), White);
    }
}

void Nodata(int x, int y, bool IconSize, String IconName)
{
    if (IconSize == LargeIcon)
        setFont(OpenSans24B);
    else
        setFont(OpenSans12B);
    drawString(x - 3, y - 10, "?", CENTER);
}

/* (C) D L BIRD
    This function will draw a graph on a ePaper/TFT/LCD display using data from an array containing data to be graphed.
    The variable 'max_readings' determines the maximum number of data elements for each array. Call it with the following parametric data:
    x_pos-the x axis top-left position of the graph
    y_pos-the y-axis top-left position of the graph, e.g. 100, 200 would draw the graph 100 pixels along and 200 pixels down from the top-left of the screen
    width-the width of the graph in pixels
    height-height of the graph in pixels
    Y1_Max-sets the scale of plotted data, for example 5000 would scale all data to a Y-axis of 5000 maximum
    data_array1 is parsed by value, externally they can be called anything else, e.g. within the routine it is called data_array1, but externally could be temperature_readings
    auto_scale-a logical value (TRUE or FALSE) that switches the Y-axis autoscale On or Off
    barchart_on-a logical value (TRUE or FALSE) that switches the drawing mode between barhcart and line graph
    barchart_colour-a sets the title and graph plotting colour
    If called with Y!_Max value of 500 and the data never goes above 500, then autoscale will retain a 0-500 Y scale, if on, the scale increases/decreases to match the data.
    auto_scale_margin, e.g. if set to 1000 then autoscale increments the scale by 1000 steps.
*/
void DrawGraph(int x_pos, int y_pos, int gwidth, int gheight, float Y1Min, float Y1Max, String title, float DataArray[], int readings, boolean auto_scale, boolean barchart_mode)
{
#define auto_scale_margin 0 // Sets the autoscale increment, so axis steps up fter a change of e.g. 3
#define y_minor_axis 5      // 5 y-axis division markers
    setFont(OpenSans10B);
    int maxYscale = -10000;
    int minYscale = 10000;
    int last_x, last_y;
    float x2, y2;
    if (auto_scale == true)
    {
        for (int i = 1; i < readings; i++)
        {
            if (DataArray[i] >= maxYscale)
                maxYscale = DataArray[i];
            if (DataArray[i] <= minYscale)
                minYscale = DataArray[i];
        }
        maxYscale = round(maxYscale + auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Max
        Y1Max = round(maxYscale + 0.5);
        if (minYscale != 0)
            minYscale = round(minYscale - auto_scale_margin); // Auto scale the graph and round to the nearest value defined, default was Y1Min
        Y1Min = round(minYscale);
    }
    // Draw the graph
    last_x = x_pos + 1;
    last_y = y_pos + (Y1Max - constrain(DataArray[1], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight;
    drawRect(x_pos, y_pos, gwidth + 3, gheight + 2, Grey);
    drawString(x_pos - 20 + gwidth / 2, y_pos - 28, title, CENTER);
    for (int gx = 0; gx < readings; gx++)
    {
        x2 = x_pos + gx * gwidth / (readings - 1) - 1; // max_readings is the global variable that sets the maximum data that can be plotted
        y2 = y_pos + (Y1Max - constrain(DataArray[gx], Y1Min, Y1Max)) / (Y1Max - Y1Min) * gheight + 1;
        if (barchart_mode)
        {
            fillRect(last_x + 2, y2, (gwidth / readings) - 1, y_pos + gheight - y2 + 2, Black);
        }
        else
        {
            drawLine(last_x, last_y - 1, x2, y2 - 1, Black); // Two lines for hi-res display
            drawLine(last_x, last_y, x2, y2, Black);
        }
        last_x = x2;
        last_y = y2;
    }
    //Draw the Y-axis scale
#define number_of_dashes 20
    for (int spacing = 0; spacing <= y_minor_axis; spacing++)
    {
        for (int j = 0; j < number_of_dashes; j++)
        { // Draw dashed graph grid lines
            if (spacing < y_minor_axis)
                drawFastHLine((x_pos + 3 + j * gwidth / number_of_dashes), y_pos + (gheight * spacing / y_minor_axis), gwidth / (2 * number_of_dashes), Grey);
        }
        if ((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing) < 5 || title == TXT_PRESSURE_IN)
        {
            drawString(x_pos - 10, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
        }
        else
        {
            if (Y1Min < 1 && Y1Max < 10)
            {
                drawString(x_pos - 3, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 1), RIGHT);
            }
            else
            {
                drawString(x_pos - 7, y_pos + gheight * spacing / y_minor_axis - 5, String((Y1Max - (float)(Y1Max - Y1Min) / y_minor_axis * spacing + 0.01), 0), RIGHT);
            }
        }
    }
    for (int i = 0; i < 3; i++)
    {
        drawString(20 + x_pos + gwidth / 3 * i, y_pos + gheight + 10, String(i) + "d", LEFT);
        if (i < 2)
            drawFastVLine(x_pos + gwidth / 3 * i + gwidth / 3, y_pos, gheight, LightGrey);
    }
}

void drawString(int x, int y, String text, alignment align)
{
    char *data = const_cast<char *>(text.c_str());
    int x1, y1; //the bounds of x,y and w and h of the variable 'text' in pixels.
    int w, h;
    int xx = x, yy = y;
    get_text_bounds(&currentFont, data, &xx, &yy, &x1, &y1, &w, &h, NULL);
    if (align == RIGHT)
        x = x - w;
    if (align == CENTER)
        x = x - w / 2;
    int cursor_y = y + h;
    write_string(&currentFont, data, &x, &cursor_y, framebuffer);
}

void fillCircle(int x, int y, int r, uint8_t color)
{
    epd_fill_circle(x, y, r, color, framebuffer);
}

void drawFastHLine(int16_t x0, int16_t y0, int length, uint16_t color)
{
    epd_draw_hline(x0, y0, length, color, framebuffer);
}

void drawFastVLine(int16_t x0, int16_t y0, int length, uint16_t color)
{
    epd_draw_vline(x0, y0, length, color, framebuffer);
}

void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color)
{
    epd_write_line(x0, y0, x1, y1, color, framebuffer);
}

void drawCircle(int x0, int y0, int r, uint8_t color)
{
    epd_draw_circle(x0, y0, r, color, framebuffer);
}

void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    epd_draw_rect(x, y, w, h, color, framebuffer);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    epd_fill_rect(x, y, w, h, color, framebuffer);
}

void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1,
                  int16_t x2, int16_t y2, uint16_t color)
{
    epd_fill_triangle(x0, y0, x1, y1, x2, y2, color, framebuffer);
}

void drawPixel(int x, int y, uint8_t color)
{
    epd_draw_pixel(x, y, color, framebuffer);
}

void setFont(GFXfont const &font)
{
    currentFont = font;
}

void edp_update()
{
    epd_draw_grayscale_image(epd_full_screen(), framebuffer); // Update the screen
}
/*
   1085 lines of code 28-01-2021
*/
