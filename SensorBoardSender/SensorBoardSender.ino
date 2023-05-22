/* SensorBoardSender.ino
 *  
 * PixelBlaze Sensor board -> Multicast datagram Sender
 * for ESP8266
 *
 * Reads data from a Pixelblaze sensor board and sends it over the 
 * network via UDP multicast.  
 * 
 * To use: 
 * Connect the ESP-01 and the sensor board to power (3v3) and ground, and connect the sensor board's
 * TX pin to the ESP-01's RX pin.
 * Set the _SSID and _PASS variables for your wifi network.
 * Compile, upload and go!
 * 
 * Requires the Arduino IDE, with the community ESP8266 library installed.
 * 
 * 6/2022 ZRanger1
 * Distributed under the MIT license
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// Serial setup constants
#define BAUD            115200            // bits/sec coming from sensor board

// Network setup constants
#define _SSID "mySSID"           // Your WiFi SSID goes here.  
#define _PASS "myPassword"         // Your WiFi password goes here.
#define _PORT 6000                // UDP port on which pbxTeleporter listens for commands

// what's in a Pixelblaze Sensor Board packet?
typedef struct {
    uint8_t  headerTag[6];  
    uint16_t frequency[32];
    uint16_t energyAverage;
    uint16_t maxFreqMagnitude;
    uint16_t maxFrequency;
    int16_t accelerometer[3];   // note: signed ints
    uint16_t light;   
    uint16_t analogInputs[5];   // note: actually 12-bit data
    uint8_t endTag[4];   
} __attribute__((packed)) SensorPacket;

// Global variables
WiFiUDP Udp;
IPAddress serverIP(224,0,0,12);  // multicast address. 
SensorPacket dataFrame;

/////////////////////////////////
// Utility Functions
/////////////////////////////////

// readBytes()
// reads the specified number of bytes into a buffer
// yields if bytes are not available
void readBytes(uint8_t *buf, uint16_t size) {
  int i = 0;
  while (i < size) {
    if (Serial.available()) {
      *buf++ = Serial.read();
      i++;
    }
    else {
      delay(0);
    }
  }  
}

// readOneByte()
// Read a single byte, yielding if the buffer is empty
uint8_t readOneByte() {
  while (!Serial.available()) {
    delay(0);
  }
  return Serial.read();
}

// readMagicWord
// reads bytes, returns true if we've found the magic header string, false otherwise
bool readMagicWord() {
  if (readOneByte() != dataFrame.headerTag[0]) return false;
  if (readOneByte() != dataFrame.headerTag[1]) return false;
  if (readOneByte() != dataFrame.headerTag[2]) return false;
  if (readOneByte() != dataFrame.headerTag[3]) return false;
  if (readOneByte() != dataFrame.headerTag[4]) return false;
  if (readOneByte() != dataFrame.headerTag[5]) return false;
 
  return true;
}

// setup()
// 
// 
// 
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(BAUD);  
  delay(250);
  Serial.setRxBufferSize(1024);

// Configure and connect WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(_SSID,_PASS);
    
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  // "magic string" that identifies a sensor board packet
  dataFrame.headerTag[0] = 'S';
  dataFrame.headerTag[1] = 'B';
  dataFrame.headerTag[2] = '1';
  dataFrame.headerTag[3] = '.';
  dataFrame.headerTag[4] = '0';    
  dataFrame.headerTag[5] = 0;      
}

// main loop
void loop() {
  
  // read data 'till we get the magic header sequence
  if (readMagicWord()) {
    // buffer the rest of the packet and send it out on the net.
    readBytes((uint8_t *) &dataFrame.frequency, sizeof(SensorPacket)-6);
    
    Udp.beginPacketMulticast(serverIP,_PORT,WiFi.localIP());        
    Udp.write((uint8_t *) &dataFrame,sizeof(SensorPacket));
    Udp.endPacket();
  }
}
