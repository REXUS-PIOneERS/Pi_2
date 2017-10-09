/*
 * Pi 2 is connected to the ImP/IMU via UART, Camera via CSI, Burn Wire Relay
 * via GPIO and Pi 1 via Ethernet and GPIO for LO, SOE and SODS signals.
 *
 * This program controls most of the main logic and for the PIOneERs mission
 * on board REXUS.
 */

#include <stdio.h>
#include <cstdint>
#include <unistd.h> //For sleep
#include <stdlib.h>
#include <iostream>

#include "RPi_IMU/RPi_IMU.h"
#include "camera/camera.h"
#include "UART/UART.h"
#include "Ethernet/Ethernet.h"
#include "pipes/pipes.h"

#include <wiringPi.h>

// Main inputs for experiment control
int LO = 29;
int SOE = 28;
int SODS = 27;

// Burn Wire Setup
int BURNWIRE = 4;

// Global variable for the Camera and IMU
PiCamera Cam = PiCamera();

// Setup for the UART communications
int baud = 230400;
UART ImP = UART();
Pipe ImP_stream;

// Ethernet communication setup and variables (we are acting as client)
int port_no = 31415; // Random unused port for communication
Pipe ethernet_stream;
Server ethernet_comms = Server(port_no);
int ALIVE = 2;

void signal_handler(int s) {
	fprintf(stdout, "Caught signal %d\n"
			"Ending child processes...", s);
	Cam.stopVideo();
	if (&ethernet_stream != NULL) {
		delay(100);
		ethernet_stream.close_pipes();
	}
	if (&ImP_stream != NULL)
		ImP_stream.close_pipes();
	fprintf(stdout, "Child processes closed, exiting program\n");
	exit(1); // This was an unexpected end so we will exit with an error!
}

int SODS_SIGNAL() {
	/*
	 * When the 'Start of Data Storage' signal is received all data recording
	 * is stopped (IMU and Camera) and power to the camera is cut off to stop
	 * shorting due to melting on re-entry. All data is copied into a backup
	 * directory.
	 */
	fprintf(stdout, "Signal Received: SODS\n");
	Cam.stopVideo();
	//ImP.stopDataCollection();
	ethernet_stream.strwrite("EXIT");
	delay(100);
	ethernet_stream.close_pipes();
	ImP_stream.close_pipes();
	return 0;
}

int SOE_SIGNAL() {
	/*
	 * When the 'Start of Experiment' signal is received the boom needs to be
	 * deployed and the ImP and IMU to start taking measurements. For boom
	 * deployment is there is no increase in the encoder count or ?? seconds
	 * have passed since start of deployment then it is assumed that either the
	 * boom has reached it's full length or something has gone wrong and the
	 * count of the encoder is sent to ground.
	 */
	fprintf(stdout, "Signal Received: SOE\n");
	// Setup the ImP and start requesting data
	ImP_stream = ImP.startDataCollection("Docs/Data/Pi2/test");
	char buf[256]; // Buffer for reading data from the IMU stream
	// Trigger the burn wire!
	digitalWrite(BURNWIRE, 1);
	unsigned int start = millis();
	fprintf(stdout, "Triggering burn wire...");
	fflush(stdout);
	while (1) {
		unsigned int time = millis() - start;
		fprintf(stdout, "%d ms ", time);
		fflush(stdout);
		if (time > 6000) break;
		// Get ImP data
		int n = ImP_stream.binread(buf, 255);
		if (n > 0) {
			buf[n] = '\0';
			fprintf(stdout, "DATA: %s\n", buf);
			ethernet_stream.binwrite(buf, n);
		}
		delay(100);
	}
	digitalWrite(BURNWIRE, 0);

	// Wait for the next signal to continue the program
	bool signal_recieved = false;
	while (!signal_recieved) {
		delay(10);
		// Implements a loop to ensure SODS signal has actually been received
		if (!digitalRead(SODS)) {
			int count = 0;
			for (int i = 0; i < 5; i++) {
				count += !digitalRead(SODS);
				delayMicroseconds(200);
			}
			if (count >= 3) signal_recieved = true;
		}

		// Read data from IMU_data_stream and echo it to Ethernet
		char buf[256];
		int n = ImP_stream.binread(buf, 255);
		if (n > 0) {
			buf[n] = '\0';
			fprintf(stdout, "DATA: %s\n", buf);
			ethernet_stream.binwrite(buf, n);
		}
	}
	return SODS_SIGNAL();
}

int LO_SIGNAL() {
	/*
	 * When the 'Lift Off' signal is received from the REXUS Module the cameras
	 * are set to start recording video and we then wait to receive the 'Start
	 * of Experiment' signal (when the nose-cone is ejected)
	 */
	fprintf(stdout, "Signal Received: LO\n");
	Cam.startVideo();
	// Poll the SOE pin until signal is received
	// TODO implement check to make sure no false signals!
	fprintf(stdout, "Waiting for SOE signal...\n");
	bool signal_received = false;
	while (!signal_received) {
		delay(10);
		// Implements a loop to ensure LO signal has actually been received
		if (!digitalRead(SOE)) {
			int count = 0;
			for (int i = 0; i < 5; i++) {
				count += !digitalRead(SOE);
				delayMicroseconds(200);
			}
			if (count >= 3) signal_received = true;
		}
		// TODO Implement communications with RXSM
	}
	return SOE_SIGNAL();
}

int main() {
	/*
	 * This part of the program is run before the Lift-Off. In effect it
	 * continually listens for commands from the ground station and runs any
	 * required tests, regularly reporting status until the LO Signal is
	 * received.
	 */
	// Setup wiringpi
	wiringPiSetup();
	// Setup main signal pins
	pinMode(LO, INPUT);
	pullUpDnControl(LO, PUD_UP);
	pinMode(SOE, INPUT);
	pullUpDnControl(SOE, PUD_UP);
	pinMode(SODS, INPUT);
	pullUpDnControl(SODS, PUD_UP);
	pinMode(ALIVE, OUTPUT);

	// Setup Burn Wire
	pinMode(BURNWIRE, OUTPUT);
	// Create necessary directories for saving files
	system("mkdir -p Docs/Data/Pi1 Docs/Data/Pi2 Docs/Data/test Docs/Video");
	fprintf(stdout, "Pi 2 is alive and running.\n");

	// Setup server and wait for client
	digitalWrite(ALIVE, 1);
	ethernet_stream = ethernet_comms.run();
	fprintf(stdout, "Waiting for LO signal...\n");

	// Check for LO signal.
	bool signal_received = false;
	while (!signal_received) {
		delay(10);
		// Implements a loop to ensure LO signal has actually been received
		if (!digitalRead(LO)) {
			int count = 0;
			for (int i = 0; i < 5; i++) {
				count += !digitalRead(LO);
				delayMicroseconds(200);
			}
			if (count >= 3) signal_received = true;
		}
		// TODO Implement communications with RXSM
	}
	return LO_SIGNAL();
}
