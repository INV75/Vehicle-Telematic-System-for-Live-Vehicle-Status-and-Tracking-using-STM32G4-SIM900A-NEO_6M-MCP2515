OVERVIEW:

The object of this project is to create a real-time remote IoT dashboard for vehicle telemetry.
This project utilizes three different communication protocols: UART, I2C and SPI.
An STM32G491RE is the microcontroller of choice, as it comes with CAN FD capabilities which can be helpful to communicate with the CAN Bus of the vehicle and adding further functionalities down the line.
The microcontroller is interfaced with a SIM900A GSM module to take the setup online utilizing its 2G-GSM communication capabilities via the HTTP protocol.
A NEO 6M module is employed for vehicle tracking, providing GPS coordinates every 1 second using Trilateration. The SIM900A also attaches a timestamp to the waypoints.
The vehicle coordinates can be plotted and tracked using Leaflet or the GPS Visualizer portal.
An MCP2515 is the CAN Transceiver module used to interface with the OBD-2 port of the vehicle with the CAN High and CAN Low pins to read incoming raw CAN data.
The IoT Dashboard is created using Node Red, wherein, gauges and text boxes and a Leaflet map has been added to the UI to configure an IoT telemetry dashboard which can be retrofitted to any vehicle.
The IoT Dashboard provides basic information right now such as the Date and Time, route tracing and vehicle running parameters such as Speed, RPM and Coolant temperature.
Currently, a Python script is being used as a placeholder to generate and run a simulated CAN 2.0B message matrix and a pre-recorded 20-km route to test the dashboard itself.
The setup is currently running in the local machine. The setup will soon be configured to transmit the data online via HTTP post.


PROJECT FILES:

The files in the main branch include STM32 projects for self-testing the breakout modules to understand and verify their working.
There are also iterations to implement different functionalities such as displaying date, time and coordinate data on a 128x64 SSD1306 OLED display and to ping the same content as a POST on a webhook url.
Other iterations include loopback tests for the NEO 6M and MCP2515 modules, pinging AT commands to the SIM900A and checking for a response, basic display testing and reading the raw CAN matrix data from the OBD-2 port of a vehicle.


CURRENT PROGRESS:
The working of the SIM900A, NEO 6M and MCP2515 have been successfully verified and can be checked using the files in the main branch of the project.

-The SIM900A is successfully receiving AT commands and giviing a response back. The connection status and AT detection is verified using the on-board LED (PA5) on the STM32 and the SSD1306.

-It is verified that the NEO 6M is working by detecting NMEA sequences transmitted by it by toggling the STM32 on-board LED (PA5).

-The CAN loopback test is successful as the MCP2515 module is giving the appropriate output in the debug mode in the "Live Expressions" tab.

-The Webhook URL is receiving POST pings from the SIM900A with 10-25 data packets of content with Date, Time, Latitude and Longitude data from the SIM900A and NEO 6M.

-The IoT dashboard is successfully created using Node Red and is receiving and displaying simulated CAN and GPS data from the local machine.
