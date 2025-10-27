☀️ IoT Solar Power Monitoring System

This project was developed as part of the Service Learning program for the Embedded Systems course at Petra Christian University.
It integrates IoT technology to monitor the performance of a solar power generation system in a community-based urban farming center (Kawasan Rumah Pangan Lestari – KWT Segaran Asri, Tambakrejo).

🚀 Project Overview

The system is designed to monitor key parameters of a photovoltaic (PV) power system in real time — including voltage, current, power, frequency, and power factor — using an ESP32 microcontroller connected to multiple sensors.
All data are transmitted to a Node-RED dashboard via MQTT, allowing users to visualize and supervise the solar system remotely through the internet.

🎯 Objectives

1. Improve efficiency and reliability in monitoring community solar energy systems.
2. Provide local operators with real-time performance data of their PV installation.
3. Implement an IoT-based monitoring architecture for sustainable community development.
4. Introduce local residents to digital technologies for renewable energy management.

🧠 System Architecture

The system continuously monitors:

1. Solar panel voltage (up to 72V)
2. Battery voltage (up to 30V)
3. AC voltage and current output from inverter

Data are read via sensors (ACS758, PZEM-004T) and processed by the ESP32, which then transmits readings every 30 seconds to an MQTT broker hosted in the cloud.
Node-RED subscribes to these MQTT topics and visualizes the data in a dashboard interface accessible online.

<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1VqjarNvZch002U2wescug45zzLD5bFT3" width="500" />
</p>

<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1msRfNeQB5b53_JbyCdMelLMCDWO5ZoVc" width="500" />
</p>

<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1jTwvTeYaSXS_f2Wwq0IKWJ6fsL13UmzC" width="500" />
</p>

🧪 Testing and Results

1. Voltage readings from both solar panel and battery were accurate within ±1V.
2. AC sensor (PZEM-004T) operated correctly, detecting voltage (220V) and loss of connection.
3. Buzzer successfully alerted when AC voltage reading failed (NaN state).
4. Current sensors (ACS758) were found faulty and excluded after overheating and inaccurate readings.

Despite this limitation, the overall system achieved stable operation and successful data transmission to Node-RED.

<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1xJBCaafB7uIVCrWh3P_nj67xVLXk7nxV" width="500" />
</p>

<p align="center">
  <img src="https://drive.google.com/uc?export=view&id=1IpG-jk9CAgEB0CG9J2misnU21oSR2Ylf" width="500" />
</p>

🌍 Impact and Sustainability

This project was implemented at KRPL KWT Segaran Asri, an urban farming community that practices sustainable energy management.
The IoT monitoring system contributes directly to community empowerment and environmental awareness by:

1. Enabling transparent energy management — local operators can see system health and performance instantly.
2. Reducing equipment failure risk — by providing early alerts on abnormal readings or inverter malfunction.
3. Supporting renewable energy education — the system serves as a live demonstration for students and visitors learning about solar technology.
4. Encouraging sustainable practices — combining IoT and solar energy promotes responsible, low-carbon community development.
