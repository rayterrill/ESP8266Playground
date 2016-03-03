/* Create a WiFi access point and provide a web server on it. */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>

const char *initialSSID = "NerdoDevice";
const int buttonPin = 0;     // the number of the pushbutton pin

int buttonState = 0;         // variable for reading the pushbutton status
byte mac[6]; // variable to hold the mac address of our device 

ESP8266WebServer server(80);

void wifiRoot() {
  String form =
  "<p>"
  "<center>"
  "<form action='setSSID'>"
  "<p>SSID<input type='text' name='ssid' size=50 autofocus></p>"
  "<p>Password (if needed)<input type='password' name='password' size=50></p>"
  "<input type='submit' value='Submit'>"
  "</form>"
  "</center>";
  
  server.send(200, "text/html", form);
}

void htmlRoot() {
  String html = "<h1>Connected to Wifi Successfully!</h1>";

  server.send(200, "text/html", html);
}

void clearEEPROM() {
  for (int i = 0; i < 96; ++i) { EEPROM.write(i, 0); }
  EEPROM.commit();
}

void handle_clearEEPROM() {
  clearEEPROM();
  delay(500);
  String response = "<p>EEPROM Cleared!</p>";
  server.send(200, "text/html", response);
}

void writeEEPROM(String ssid, String password) {
  clearEEPROM();
  Serial.println("Writing ssid to EEPROM...");
  for (int i = 0; i < ssid.length(); ++i) {
    EEPROM.write(i, ssid[i]);
  }
  Serial.println("Writing password to EEPROM..."); 
  for (int i = 0; i < password.length(); ++i) {
    EEPROM.write(32+i, password[i]);
  }    
  EEPROM.commit();
}

void handle_msg() {
  String userSSID = server.arg("ssid");
  String userPassword = server.arg("password");
  String response = "<p>SSID data saved to EEPROM. Restart device to pick up new settings.</p>";

  writeEEPROM(userSSID, userPassword);
  server.send(200, "text/html", response);
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); 
}

bool testWifi(void) {
  int c = 0;
  Serial.print("Waiting for Wifi to connect...");
  while ( c < 20 ) {
    if (WiFi.status() == WL_CONNECTED) { return true; } 
    delay(500);
    Serial.print(WiFi.status());    
    c++;
  }
  Serial.println("");
  Serial.println("Unable to connect to wifi with the given settings. Clearing EEPROM...");
  clearEEPROM();
  return false;
}

void enableWifiPages() {
  server.on("/", wifiRoot);
  server.on("/setSSID", handle_msg);
  server.on("/clear", handle_clearEEPROM);
  server.begin();
  Serial.println("HTTP server started");  
}

void buildAP() {
  WiFi.macAddress(mac);
  Serial.print("Last 4 of Mac Address: ");
  Serial.print(mac[4],HEX);
  Serial.println(mac[5],HEX);
  
  String AP_NameString = initialSSID + String(mac[4], HEX) + String(mac[5], HEX); // append the last two octets to the ssid constant to get a pseudo-random ssid
  Serial.print("Setting Access Point to: ");
  Serial.println(AP_NameString.c_str());
  WiFi.softAP(AP_NameString.c_str());

  IPAddress myIP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(myIP);

  enableWifiPages();
}

void enableNormalPage() {
  //remove me
  server.on("/", htmlRoot);
  server.on("/clear", handle_clearEEPROM);
  server.begin();
}

void connectToWifi(String ssid, String password) {
  Serial.print("Connecting to ssid: ");
  Serial.println(ssid.c_str());
  Serial.print("With password: ");
  Serial.println(password.c_str());
  WiFi.begin(ssid.c_str(), password.c_str());

  delay(10000);

  if (testWifi()) {
    Serial.println("Successfully connected to wifi!");
    enableNormalPage();
  } else {
    /* failed to connect */
    Serial.println("Launching AP...");
    buildAP();
  }
}

void setup() {
  delay(100);
	Serial.begin(115200);
  EEPROM.begin(512);
	Serial.println();

  //initialize the gpio pin as input
  pinMode(buttonPin, INPUT);

  String esid;
  for (int i = 0; i < 32; ++i) {
      esid += char(EEPROM.read(i));
  }
  String epass = "";
  for (int i = 32; i < 96; ++i) {
      epass += char(EEPROM.read(i));
  }
  if (esid == NULL) {
    Serial.println("SSID from EEPROM IS NULL.");
	  Serial.print("Configuring access point...");
    Serial.println();

    buildAP();
  } else {
    Serial.print("Read SSID from EEPROM: ");
    Serial.println(esid);
    connectToWifi(esid, epass);
  }
}

void loop() {
	server.handleClient();

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // code to handle erasing the EEPROM
  int erase = 0;
  int eraseOccured = 0;
  while (buttonState == LOW) {
    buttonState = digitalRead(buttonPin);
    erase++;
    if (erase >= 1000000) {
      Serial.print("Erasing the EEPROM as requested. ");
      clearEEPROM();
      WiFi.disconnect();
      erase = 0;
      eraseOccured = 1;
      Serial.println("Finished...");
      delay(2000);
    }
  }
}
