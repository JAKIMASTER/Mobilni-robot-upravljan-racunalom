#include <Arduino.h>
#include <Servo.h>

#define CRSF_ADDRESS 0xC8
#define CRSF_FRAMETYPE_RC_CHANNELS 0x16

uint8_t buffer[64];
uint8_t buf_pos = 0;
uint16_t channels[16];

Servo steerServo;   // Pin 9
Servo escMotor;     // Pin 10
Servo yawServo;     // Pin 6
Servo pitchServo;   // Pin 5

unsigned long lastPacketTime = 0;
unsigned long lastWriteTime = 0;
const unsigned long WRITE_INTERVAL_MS = 20;

int targetSteer = 1500;
int targetThrottle = 1500;
int targetYaw = 1500;
int targetPitch = 1500;

bool gotFirstFrame = false;

int mapCRSFtoPWM(int v) {
    return constrain(map(v, 172, 1811, 1000, 2000), 1000, 2000);
}

void decodeCRSFChannels(uint8_t *payload) {
    uint32_t bitbuf = 0;
    uint8_t bits = 0;
    uint8_t ch_index = 0;
    for (int i = 0; i < 22; i++) {
        bitbuf |= ((uint32_t)payload[i]) << bits;
        bits += 8;
        while (bits >= 11 && ch_index < 16) {
            channels[ch_index] = bitbuf & 0x7FF;
            bitbuf >>= 11;
            bits -= 11;
            ch_index++;
        }
    }
}

void setup() {
    Serial.begin(115200);
    steerServo.attach(9);
    escMotor.attach(10);
    yawServo.attach(11);
    pitchServo.attach(2);

    steerServo.writeMicroseconds(1500);
    escMotor.writeMicroseconds(1500);
    yawServo.writeMicroseconds(1500);
    pitchServo.writeMicroseconds(1500);
}

void loop() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        buffer[buf_pos++] = b;
        if (buf_pos >= 64) buf_pos = 0;

        if (buffer[0] == CRSF_ADDRESS && buf_pos >= 2) {
            uint8_t length = buffer[1];
            uint8_t total_packet_size = length + 2; 

            if (buf_pos >= total_packet_size) {
                if (buffer[2] == CRSF_FRAMETYPE_RC_CHANNELS) {
                    decodeCRSFChannels(&buffer[3]);
                    lastPacketTime = millis();
                    gotFirstFrame = true;

                    targetSteer    = mapCRSFtoPWM(channels[0]);
                    targetThrottle = mapCRSFtoPWM(channels[1]);
                    targetYaw      = mapCRSFtoPWM(channels[2]); 
                    targetPitch    = mapCRSFtoPWM(channels[3]);
                }

                memmove(buffer, buffer + total_packet_size, buf_pos - total_packet_size);
                buf_pos -= total_packet_size;
            }
        } else if (buf_pos > 0 && buffer[0] != CRSF_ADDRESS) {
            buf_pos = 0;
        }
    }

    unsigned long now = millis();
    if (now - lastPacketTime > 500) {
        targetThrottle = 1500;
        gotFirstFrame = false;
    }

    if (now - lastWriteTime >= WRITE_INTERVAL_MS) {
        lastWriteTime = now;
        if (gotFirstFrame) {
            steerServo.writeMicroseconds(targetSteer);
            escMotor.writeMicroseconds(targetThrottle);
            yawServo.writeMicroseconds(targetYaw);
            pitchServo.writeMicroseconds(targetPitch);
        }
    }
}
