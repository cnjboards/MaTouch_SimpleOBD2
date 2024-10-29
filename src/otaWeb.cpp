// Includes
#include "stationId.h"
#include <Arduino.h>
#include "otaWeb.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

// Globals
bool startUpDelayDone=false;
bool fileUploadStarted=false;

const char *host = "ESP32-S3";
#if defined(STATION_A)
    const char *ssid = "n2kMaTouch21AP";
#else
    const char *ssid = "n2kMaTouch128AP";
#endif
const char *password = "thisisfine";

// used for ota updates
WebServer server(80);

// used to uniquely identify AP
extern u_int32_t chipId;

// used for ota web stuff
void onJavaScript(void) {
    Serial.println("onJavaScript(void)");
		server.setContentLength(jquery_min_js_v3_2_1_gz_len);
		server.sendHeader(F("Content-Encoding"), F("gzip"));
    server.send_P(200, "text/javascript", jquery_min_js_v3_2_1_gz, jquery_min_js_v3_2_1_gz_len);
}

/* setup function */
void otaSetup(void)
{
    // create a unique ssid based on constant + chipid
    char locBuf[128], locChipId[16];
    strcpy(locBuf, ssid);
    sprintf(locChipId,"-%x", chipId);
    strcat(locBuf, locChipId);
    
    IPAddress local_ip(192, 168, 1, 1);
    IPAddress local_mask(255,255,255,0);
    IPAddress gateway(192, 168, 1, 1);

    WiFi.softAP(locBuf,password);
    WiFi.softAPConfig(local_ip,gateway,local_mask);

    /*use mdns for host name resolution*/
    if (!MDNS.begin(host))
    { // http://esp32.local
        Serial.println("Error setting up MDNS responder!");
        while (1)
        {
            delay(1000);
        }
    }
    Serial.println("mDNS responder started");

    /*return javascript jquery */
    server.on("/jquery.min.js", HTTP_GET, onJavaScript);

    /*return index page which is stored in serverIndex */
    server.on("/", HTTP_GET, []()
              {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex); });
    server.on("/serverIndex", HTTP_GET, []()
              {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex); });
    /*handling uploading firmware file */
    server.on(
        "/update", HTTP_POST, []()
        {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart(); },
        []()
        {
            HTTPUpload &upload = server.upload();
            if (upload.status == UPLOAD_FILE_START)
            {
                Serial.printf("Update: %s\n", upload.filename.c_str());
                fileUploadStarted = true;
                if (!Update.begin(UPDATE_SIZE_UNKNOWN))
                { // start with max available size
                    Update.printError(Serial);
                }
            }
            else if (upload.status == UPLOAD_FILE_WRITE)
            {
                /* flashing firmware to ESP*/
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
                {
                    Update.printError(Serial);
                    fileUploadStarted = false;
                }
            }
            else if (upload.status == UPLOAD_FILE_END)
            {
                if (Update.end(true))
                { // true to set the size to the current progress
                    Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
                    fileUploadStarted = false;
                }
                else
                {
                    Update.printError(Serial);
                    fileUploadStarted = false;
                } // end if
            } // end if
        });
    server.begin();
} // end otasetup