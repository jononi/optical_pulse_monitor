/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "application.h"
#line 1 "c:/Users/jaafe/OneDrive/Particle_workspace/optical_pulse_monitor/src/optical_pulse_monitor.ino"
/*
 * Optical_pulse_monitor
 * Description: V1.0 Display pulse from PPG sensor and estimates heartbeat
 * V1.1   added estimate of HB using the Red LED 
 * V1.2   adding more robust HB rate estimate
 * + increase effective sampling rate to 50 SPS and sample time to 5s
 * future V2.x   estimate BP from pulse signal analysis 
 * future V3.x   add pedometer function + motion compensation
 *
 */

void setup();
void loop();
void syncUpdatesHandler();
void modeButtonHandler(system_event_t event, int data);
void top_status_bar_update();
void display_update(uint8_t sample_i);
void ble_transfer(int *int_array1, int *int_array2);
void ble_transfer(float *float_arrayR, float *float_arrayIR, int array_length);
void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
int batteryStatus(String command);
int getSignalQuality(String cmd);
#line 12 "c:/Users/jaafe/OneDrive/Particle_workspace/optical_pulse_monitor/src/optical_pulse_monitor.ino"
#define SAMPLE_PERIOD_MS    15
#define SYNC_PERIOD_MS      400
#define AUTOSLEEP_MS        120000

#define RED_SIGNAL 1
#define IR_SIGNAL  0

#if (PLATFORM_ID == PLATFORM_XENON)
	#define xenon
#elif	(PLATFORM_ID == PLATFORM_BORON)
	#define boron
#elif	(PLATFORM_ID == PLATFORM_ARGON)
	#define argon
#endif

#include    "Adafruit_SSD1306_RK.h"
#include    "FreeSansBoldOblique9pt7b.h"   
#include    "icons.h"
#include    "MAX30105.h"
#include    "Pulse_processing.h"

#ifdef boron
#include	"cellular_hal.h"
#define 	hologram_sim
#endif

SYSTEM_MODE(SEMI_AUTOMATIC);
SYSTEM_THREAD(ENABLED);

const char *appName = "Pulse Monitor";

// status flags
bool p_sensor_on        = false;
// used to stop display refresh during pulse measurement to avoid I2C conflicts and acheieve target rate 
bool display_on         = true;
bool draw_pulse         = false;
bool enter_power_save_mode   = false;
bool power_save_mode    = false;
bool power_off_mode     = false;
bool run_wakeup_sequence = false;
bool toggle_mesh_interface = false;

// update flags
bool update_display    = false;
bool sample_p_sensor   = false;
bool transmit_ble      = false;

// operating variables
int     pulse_sample_n = 0;  // length of samples array
uint8_t sync_token = 0;
uint8_t sample_index = 0;
char    TX_buffer[64];

#ifdef boron
FuelGauge	battery;
CellularSignal sig;
STARTUP(System.enableFeature(FEATURE_RETAINED_MEMORY));
#endif

// target variables
int32_t irValue = 0;
int32_t redValue = 0;

// debug purpose
float r_xxR[52], r_xxIR[52];
      
// Timers
Timer syncUpdatesTimer(SYNC_PERIOD_MS, syncUpdatesHandler);
Timer ps_sample_timer(SAMPLE_PERIOD_MS, []() { sample_p_sensor = true; });
Timer autoSleep_timer(AUTOSLEEP_MS, []() { enter_power_save_mode = true; }, true);

// Instances
Adafruit_SSD1306 display(128, 64, -1); // I2C mode, No reset pin
MAX30105 particleSensor;
Pulse_processing signal_process;

// Cellular patameters


// BLE parameters
// BleSerialPeripheralStatic<64, 64> bleSerial;
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY, txUuid, serviceUuid);
BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP, rxUuid, serviceUuid, onDataReceived, NULL);


enum sleepModeType {powersave, poweroff};
void sleep_sequence(sleepModeType sleep_mode);

SerialLogHandler logHandler(115200, LOG_LEVEL_WARN, {
    { "app", LOG_LEVEL_INFO },
});

void setup() {
    delay(100); 	
	#ifdef boron
	battery.wakeup();
	battery.quickStart();
	#endif

    Serial.begin();  
    // initialize particle sensor
    p_sensor_on = particleSensor.begin(Wire, I2C_SPEED_FAST);
    if (p_sensor_on) {
        //effective sampling rate is 50 sps = 100sps / 2 
        byte ledBrightness = 32; //Options: 0=Off to 255=51 mA.  32 => ~6.2mA
        byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
        byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
        int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200 sps
        int pulseWidth = 411; //Options: 69, 118, 215, 411 us. Determines the ADC resolution: 15, 16, 17, 18 bits
        int adcRange = 4096; //Full scale(saturation) in nA for 18 bits ADC. Options: 2048, 4096, 8192, 16384
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings 
        // particleSensor.setPulseAmplitudeRed(16); //dim red LED since it only indicates sensor is running
        // particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
        signal_process.setup(); // initialize helper arrays
        pulse_sample_n = signal_process.get_sample_length();
    }


    // Splash screen
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    display.clearDisplay();
    display.drawBitmap(32, 16, pulse, 32, 32, 1);
    display.setTextSize(1);
    display.setTextColor(WHITE, BLACK); // redraw background
    display.setFont(&FreeSansBoldOblique9pt7b);
    display.setCursor(3, 63);
    display.print(appName);
    display.setFont();
    
    display.display();  

    // register MODE button event
    System.on(button_final_click, modeButtonHandler);

    // Init BLE peripheral
    // bleSerial.setup("PulseMon");
    BLE.addCharacteristic(txCharacteristic);
    BLE.addCharacteristic(rxCharacteristic);
    BleAdvertisingData data;
    data.appendServiceUUID(serviceUuid);
    data.appendLocalName("Pulse");
    BLE.advertise(&data);

    // Cloud functions
    Particle.function("Get_Battery", batteryStatus);
	Particle.function("getSigQual", getSignalQuality);

    // local time settings
	Time.zone(-5);
	Time.setDSTOffset(1.0);
	Time.beginDST();

    #ifdef boron
	Cellular.on();

    delay(200);
	Cellular.connect();
	#endif

    // Particle.connect();
    delay(500);

    // start timers
    ps_sample_timer.start();
    syncUpdatesTimer.start();
}

uint32_t ticker;

void loop() {
    if (sample_p_sensor) {
        particleSensor.getIRandRed(&irValue, &redValue);
        // irValue = particleSensor.getIR();
        // redValue = particleSensor.getRed();

        if (irValue > 80000 && sample_index < pulse_sample_n) {
            // starting hold on period for pulse sampling
            if (sample_index == 0) {
                update_display = false;
                display_on = false;
                // sampling on progress
                display.fillRect(0, 14, 128, 50,BLACK); // partial clear
                display.setCursor(3,18);
                display.print("keep finger pressed");
                display.setCursor(3, 38);
                display.printf("ir#: %lu", irValue);
                display.drawBitmap(104, 28, heartbeat_small, 24, 21, WHITE);
                display.display();
                delay(1000); //allow time for finger to settle down
                Log.info("Data acquisition has started...");
                ticker = millis();
                // repeat sampling since the previous one became too old because of dsiplay refresh
                irValue = particleSensor.getIR();
                redValue = particleSensor.getRed();
            }

            // update the buffers
            signal_process.ir_signal[sample_index] = irValue;
            signal_process.red_signal[sample_index] = redValue;
            sample_index++;

            // if buffers full end hold on period
            if (sample_index == pulse_sample_n) {
                Log.info("%i samples collected in %lu ms.", pulse_sample_n, millis() - ticker);
                Log.info("centering and linear trend compensation for IR signal.");
                signal_process.centering(IR_SIGNAL);
                signal_process.remove_linear_trend(IR_SIGNAL);
                Log.info("centering and linear trend compensation for RED signal.");
                signal_process.centering(RED_SIGNAL);
                signal_process.remove_linear_trend(RED_SIGNAL);
                
                // Debug
                float r_0_R  = signal_process.autocorrelation(RED_SIGNAL, 0);
                float r_0_IR = signal_process.autocorrelation(IR_SIGNAL, 0);
                for (int lag=1;lag<53;lag++) {
                    if (lag>21) {
                        r_xxR[lag-1] = signal_process.autocorrelation(RED_SIGNAL, lag)/r_0_R;
                        r_xxIR[lag-1] = signal_process.autocorrelation(IR_SIGNAL, lag)/r_0_IR;
                    } else {
                        r_xxR[lag-1] = 1.0;
                        r_xxIR[lag-1] = 1.0;
                    }
                    // Log.info("r_xx_R[%d] = %.3f : %.1f bpm", lag, r_xxR[lag], (float)(3000/lag));
                }
                /*for (int i = 0;i<pulse_sample_n;i++) {
                    Serial.printf("%i,%i\n",signal_process.red_signal[i], signal_process.ir_signal[i]);
                }*/
                
                if (BLE.connected()) transmit_ble = true;
                display_on = true;
                update_display = true;
            }
        } else if (irValue < 20000) {
            // if data acquisition session was on going, print how much progress made so far
            if (sample_index > 0) Log.info("data aq interrupted at sample %d", sample_index);
            // finger lifted up, reset buffer position
            sample_index = 0;
            // re-enable display refresh
            display_on = true; 
        }
        sample_p_sensor = false;
    }
    
    if (update_display) {
		// uint32_t time_to_refresh_display = millis();
   		// Log.info("-%lu- : screen updated in %lu ms\n", millis(), millis() - time_to_refresh_display);
        display_update(sample_index);
        if (sample_index == pulse_sample_n) {     
            sample_index = 0; // reset buffer for another round of sampling
        }
        update_display = false;
    }

    if (transmit_ble) {
        ble_transfer(r_xxR, r_xxIR, 52);
        // ble_transfer(signal_process.red_signal, signal_process.ir_signal);
        transmit_ble = false;
    }
    
    if (!Time.isValid() && Particle.connected()) Particle.syncTime();

    if (enter_power_save_mode) {
        sleep_sequence(sleepModeType::powersave); 
        power_save_mode = true;
        enter_power_save_mode = false; 
    }

    if (power_off_mode) {
        sleep_sequence(sleepModeType::poweroff);
    }

    if (run_wakeup_sequence) {
        // reset the particle sensor
        particleSensor.softReset();
        // particleSensor.setPulseAmplitudeRed(32);
        // particleSensor.setPulseAmplitudeIR(32);
    }

    if (toggle_mesh_interface) {
        display.clearDisplay();
        display.setCursor(18, 20);
        if (Mesh.ready()) {
            Mesh.off();
            display.print("Mesh Off");
        } else {
            Mesh.on();
            display.print("Mesh On");
            Particle.connect();
        }
        display.display();
        toggle_mesh_interface = false;
    }
}


void syncUpdatesHandler() {
    if (display_on) update_display = true;
    /*
    	sync_token++;
    // 200 ms updates
    if (sync_token % 2 == 0 ) {}
	// 500 ms (2 fps) updates
    if (sync_token % 5 == 0 ) {
        if (display_on) update_display = true;
    }
	// 1 sec updates
	if (sync_token == 10) {
		sync_token = 0; // reset token every 1 sec
	}
     */
}

void modeButtonHandler(system_event_t event, int data) {
	uint8_t nb_clicks = system_button_clicks(data);
	switch (nb_clicks) {
		// turn on/off mesh, connect to cloud
		case 1:
            toggle_mesh_interface = true;
			break;
		// power off
		case 2:
            power_off_mode = true;
			break;
		// experimental: low power mode
		case 3: 
        	if (!power_save_mode) enter_power_save_mode = true;
            else run_wakeup_sequence = true;
			break;
		default:
		break;
	}	
}

void top_status_bar_update() {
	display.setCursor(0, 0);
    // connectivity status
	if(Particle.connected()) {
		display.drawBitmap(0, -1, cloud_icon, 10, 10, 1);
	} else {
		#ifdef boron
		if(Cellular.ready()) {
			display.drawBitmap(0, -1, signal_4G_icon, 12, 12, WHITE);
		} else {
			display.drawBitmap(0, -1, signal_off_icon, 12, 12, WHITE);
		}
		#elif defined(xenon)
		if (Mesh.ready()) {
			display.drawBitmap(0, -1, graph_icon, 12, 12, 1);
		} else {
			display.print("..");
		}
		#elif defined(argon)
		if (WiFi.ready()) {
			display.drawBitmap(0, -1, wifi_icon, 12, 12, 1);
		} else {
			display.print("..");
		}
		#endif
	}
    // particle sensor status
    if(!p_sensor_on) {
        display.drawBitmap(18, -1, alert_icon, 12, 12, 1);
	}
	// current local time
	if (Time.isValid()) {
		display.setCursor(44, 0);
		display.printf("%d:%02d%s", Time.hourFormat12(), Time.minute(), Time.isAM()?"AM":"PM");
	}
    // BLE status
    if(BLE.connected()) {
        display.drawBitmap(88, 0, bluetooth_icon, 12, 12, 1);
    }
	// battery voltage (xenon) or charge percentage (boron)
	display.setCursor(104, 0);
	#if defined(xenon) || defined(argon)
	float battV = analogRead(BATT) * 0.0011224;
	display.printf("%.1fv", battV);
	#elif defined(boron)
	float soc = battery.getSoC();
	display.printf("%.0f%%", soc<100.0?soc:100.0);
	#endif
}

void display_update(uint8_t sample_i) {
    display.clearDisplay();
	top_status_bar_update();
    if (p_sensor_on) {
        // sample captured and processed. Now draw it.
        if (sample_i == pulse_sample_n) {
            Log.info("drawing pulse...");
            display.drawRect(0, 14, 102, 50, WHITE);
            // we can only draw 100 values, regardless of sample length
            for (int16_t index= 0;index<100; index++) {
                signal_process.remap_for_drawing(IR_SIGNAL);
                int index_y;
                if (pulse_sample_n == 100) index_y = index;
                else {
                    index_y = (int)(2*index);
                }
                int16_t pix_y = (int16_t)(64 - signal_process.drawn_signal[index_y]);
                display.drawPixel(index+1, pix_y, WHITE);
            }
            // display bpm and quality
            Log.info("calculate heart rate...");
            int heart_rate;
            float quality;
            quality = signal_process.get_bpm(IR_SIGNAL, &heart_rate);
            Log.info("IR HR: %i, Q: %.2f", heart_rate, quality);

            display.setCursor(104, 20);
            display.print(heart_rate);
            display.setCursor(104, 30);
            display.printf("%.2f", quality>0?quality:0.0);

            quality = signal_process.get_bpm(RED_SIGNAL, &heart_rate);
            Log.info("RED HR: %i, Q: %.2f", heart_rate, quality);

            display.setCursor(104, 45);
            display.print(heart_rate);
            display.setCursor(104, 55);
            display.printf("%.2f", quality>0?quality:0.0);
        }
        // ready to take samples
        if (sample_i == 0) {
            display.setCursor(3,28);
            display.printf("ir#: %lu", irValue);
            display.setCursor(3,38);
            display.print("place finger tip");
            display.setCursor(3,48);
            display.print("on sensor"); 
        }
        /*
        display.setCursor(104, 42);
        display.print("120");
        display.setCursor(106, 52);
        display.print("BPM");
        */
    } else {
        display.setCursor(0,20);
        display.print("pulse sensor error");
    }
    display.display();
    if (sample_i == pulse_sample_n && !BLE.connected()) delay(2000);
}

void sleep_sequence(sleepModeType sleep_mode) {
    syncUpdatesTimer.stop();
    autoSleep_timer.stop();
	display.clearDisplay();
	display.setCursor(15,8);
    if (sleep_mode == sleepModeType::powersave) {
        display.print("Sleep mode...");
        particleSensor.shutDown();
        // shutdown particle sensor's LEDs to save power
        // particleSensor.setPulseAmplitudeRed(0);
        // particleSensor.setPulseAmplitudeIR(0);
    } else {
        display.print("Power Off...");
        particleSensor.shutDown();
        // Mesh.off();
        // BLE.off(); //BLE.end()
    }   
	display.display();
	delay(3000); // allow plenty of times to power off radio modules and get minimum power usage
	display.clearDisplay();
	display.display();

	if (sleep_mode == sleepModeType::poweroff) {
        display.ssd1306_command(SSD1306_DISPLAYOFF);
        System.sleep(SLEEP_MODE_DEEP);
    }
}

/**************************************************
 *                   BLE 
**************************************************/

// so far it looks like Serial BLE transfer is at  ~10 points / seconds
void ble_transfer(int *int_array1, int *int_array2) {
    // send samples by tuples of 4 to speed up transmission: 24 bytes max per payload)
    for (uint8_t index = 0; index<pulse_sample_n;index++) {
        // sprintf(TX_buffer,"%lu\n%lu\n%lu\n%lu\n",irValue_vals[index], irValue_vals[index+1], irValue_vals[index+2], irValue_vals[index+3]);
        sprintf(TX_buffer,"%i,%i\n", int_array1[index], int_array2[index]);
        String sample_s = String(TX_buffer);
        txCharacteristic.setValue(sample_s);
    } 
}

void ble_transfer(float *float_arrayR, float *float_arrayIR, int array_length) {
    for (uint8_t index = 0; index<array_length;index++) {
        sprintf(TX_buffer,"%.3f,%.3f\n", float_arrayR[index], float_arrayIR[index]);
        String sample_s = String(TX_buffer);
        txCharacteristic.setValue(sample_s);
    } 
}    

void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
    // Log.trace("Received data from: %02X:%02X:%02X:%02X:%02X:%02X:", peer.address()[0], peer.address()[1], peer.address()[2], peer.address()[3], peer.address()[4], peer.address()[5]);
    for (size_t ii = 0; ii < len; ii++) {
        Serial.write(data[ii]);
    }
}

/**************************************************
 *                   Cloud functions
**************************************************/

int batteryStatus(String command){
	// Publish the battery charge percentage
  	char  buffer[64];
	#if defined(xenon) || defined(argon)
	float battV = analogRead(BATT) * 0.0011224;
	sprintf(buffer, "battery_V=%.1fv", battV);
	Log.info(buffer);
	Particle.publish(appName, buffer, 60, PRIVATE);
	return (int)(battV*1000);
	#elif defined(boron)
	float batteryV	 = battery.getVCell();
  	float batterySoc = battery.getSoC();
	sprintf(buffer, "{\"battery\":{\"charge\":%.2f,\"voltage\":%.2f }}", batterySoc, batteryV);
	Log.info(buffer);
	Particle.publish(appName, buffer, 60, PRIVATE);
	return (int)batterySoc;
	#endif

}

int getSignalQuality(String cmd) {
    char  buffer[64];
	#ifdef boron
	sig = Cellular.RSSI();
	snprintf(buffer, sizeof(buffer), "{\"signal\":{\"rssi\":%d,\"qual\":%d }}", sig.rssi, sig.qual);
	if (Particle.connected()){
		Particle.publish(appName, buffer, 60, PRIVATE);
	}
	return (int)sig.rssi;
	#elif defined(xenon)
	if (Particle.connected()){
		Particle.publish(appName, "mesh:on", 60, PRIVATE);
	}
	return 100;
	#elif defined(argon)
	WiFiSignal sig = WiFi.RSSI();
	snprintf(buffer, sizeof(buffer), "{\"signal\":{\"rssi\":%d,\"qual\":%d }}", sig.rssi, sig.qual);
	if (Particle.connected()) {
		Particle.publish(testedDeviceName, buffer, 60, PRIVATE);
	}
	return (int)sig.rssi;
	#endif		
}
