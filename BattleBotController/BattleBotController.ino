#include <Bluepad32.h>
#include <CytronMotorDriver.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
CytronMD motorR(PWM_DIR,15, 2);
CytronMD motorL(PWM_DIR,0, 4);





//F1§F2 - front motors tо driver 1
//R1§R2 - rear motors tо driver 2
//Connect common ground to each driver Mite
int on = 0;
int acc = 0;
const int deadzone = 50;
const int accSpeed = 204;

int interval = 5000;
time_t prev = 0;

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            // Additionally, you can get certain gamepad properties like:
            // Model, VID, PID, BTAddr, flags, etc.
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n BTAddr: %02x:%02x:%02x:%02x:%02x:%02x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id, 
                           properties.btaddr[0], properties.btaddr[1], properties.btaddr[2], properties.btaddr[3], properties.btaddr[4], properties.btaddr[5]);
            // Define the desired Bluetooth address
            uint8_t desiredBtAddr[] = {0xa4, 0xae, 0x12, 0xeb, 0xf9, 0x73};
            
            // Compare the Bluetooth address with the desired address
            if (memcmp(properties.btaddr, desiredBtAddr, 6) == 0) {
                Serial.println("Desired Bluetooth address matched!");
                myControllers[i] = ctl;
                foundEmptySlot = true;
                break;
            } else {
                Serial.println("Desired Bluetooth address not matched!");
            }

            
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not found empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    bool foundController = false;

    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            foundController = true;
            break;
        }
    }

    if (!foundController) {
        Serial.println("CALLBACK: Controller disconnected, but not found in myControllers");
    }
}

void dumpGamepad(ControllerPtr ctl) {
    Serial.printf(
        "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
        "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
        ctl->index(),        // Controller Index
        ctl->dpad(),         // DPAD
        ctl->buttons(),      // bitmask of pressed buttons
        ctl->axisX(),        // (-511 - 512) left X Axis
        ctl->axisY(),        // (-511 - 512) left Y axis
        ctl->axisRX(),       // (-511 - 512) right X axis
        ctl->axisRY(),       // (-511 - 512) right Y axis
        ctl->brake(),        // (0 - 1023): brake button
        ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
        ctl->miscButtons(),  // bitmak of pressed "misc" buttons
        ctl->gyroX(),        // Gyro X
        ctl->gyroY(),        // Gyro Y
        ctl->gyroZ(),        // Gyro Z
        ctl->accelX(),       // Accelerometer X
        ctl->accelY(),       // Accelerometer Y
        ctl->accelZ()        // Accelerometer Z
    );
}



void processGamepad(ControllerPtr ctl) {
    // There are different ways to query whether a button is pressed.
    // By query each button individually:
    //  a(), b(), x(), y(), l1(), etc...
    if (ctl->a()) {
      if (millis() - prev >= interval) {
        prev = millis();
        if(on == 0){
          digitalWrite(19, HIGH);
          on = 1;
        }
        else{
          digitalWrite(19, LOW);
          on = 0;
        }
        Serial.println(on);
      }
      else{
        Serial.println("time has NOT passed");
      }
        
    }

    if (ctl->b()) {
        
    }

    if (ctl->x()) {
        // Duration: 255 is ~2 seconds
        // force: intensity
        // Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
        // rumble.
        // It is possible to set it by calling:
        ctl->setRumble(0xc0 /* force */, 0xc0 /* duration */);
    }

    if ((acc - accSpeed) * -1 <= ctl->brake() && ctl->throttle() == 0){
      acc -= accSpeed;
    }
    else if (acc * -1 > ctl->brake() && ctl->throttle() == 0){
      acc += accSpeed;
      if (acc * -1 < deadzone){acc = 0;}
    }

    else if (acc + accSpeed <= ctl->throttle() && ctl->brake() == 0){
      acc += accSpeed;
    }
    else if (acc > ctl-> throttle() && ctl->brake() == 0){
      acc -= accSpeed;
      if (acc < deadzone){acc = 0;}
    }
    //Serial.println(acc);
    int speed = map(acc, -1020, 1020, -220, 220);
    speed *= -1;
    //Serial.println(speed);
    //Serial.println(static_cast<int>((acc/10)*2.4));

    if(ctl->axisX() < 0 - deadzone){
      motorR.setSpeed(speed * -1);
      motorL.setSpeed(speed);
      Serial.println("steer left");
    }
    else if(ctl->axisX() > 0 + deadzone){
      motorR.setSpeed(speed);
      motorL.setSpeed(speed * -1);
      Serial.println("steer right");
    }
    else{
      motorR.setSpeed(speed);
      motorL.setSpeed(speed);
    }
    //dumpGamepad(ctl); //print values of ps4 to serial monitor

}

// Arduino setup function. Runs in CPU 1
void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
    
    // Setup the Bluepad32 callbacks
    BP32.setup(&onConnectedController, &onDisconnectedController);

    // "forgetBluetoothKeys()" should be called when the user performs
    // a "device factory reset", or similar.
    // Calling "forgetBluetoothKeys" in setup() just as an example.
    // Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
    // But might also fix some connection / re-connection issues.
    BP32.forgetBluetoothKeys();
    // When enabled controllers like DualSense and DualShock4 generate two connected devices:
    // - First one: the gamepad
    // - Second one, which is a "vritual device", is a mouse
    // By default it is disabled.
    BP32.enableVirtualDevice(false);

    pinMode(19, OUTPUT);
    digitalWrite(19, LOW);
    motorR.setSpeed(0);
    motorL.setSpeed(0);
    
}





// Arduino loop function. Runs in CPU 1
void loop() {
    // This call fetches all the gamepad info from the NINA (ESP32) module.
    // Just call this function in your main loop.
    // The gamepads pointer (the ones received in the callbacks) gets updated
    // automatically.
    BP32.update();

    // It is safe to always do this before using the gamepad API.
    // This guarantees that the gamepad is valid and connected.
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        ControllerPtr myController = myControllers[i];

        if (myController && myController->isConnected()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.printf("Data not available yet\n");
                continue;
            }
            // See ArduinoController.h for all the available functions.
        }
    }
    
    // vTaskDelay(1);
    delay(150);
}
