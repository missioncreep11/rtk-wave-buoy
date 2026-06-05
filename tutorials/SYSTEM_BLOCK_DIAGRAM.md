# RTK Wave Buoy — System Block Diagram

This diagram shows the hardware components, communication links, and data flow of the RTK GPS wave buoy system.

```mermaid
flowchart TD
    subgraph BUOY["Buoy Hardware"]
        ANT["ANN-MB1\nGNSS Antenna"] -->|RF| ZED["ZED-F9P\nRTK Receiver"]
        ZED <-->|"I2C / Qwiic"| OLA["OpenLog Artemis\n(Apollo3 + ICM-20948 IMU)"]
        ESP["ESP32\nNTRIP Client"] -->|"UART 115200 baud\nRTCM3 corrections"| ZED
    end

    subgraph CLOUD["Internet"]
        POL["Polaris Network RTK\npolaris.pointonenav.com"]
    end

    subgraph OUTPUT["Data Output"]
        SD["MicroSD Card\ndataLog.ubx  ·  imuLog.csv"]
    end

    ESP <-->|"WiFi / LTE\n(NTRIP protocol)"| POL
    OLA -->|SPI| SD
```

## Component Summary

| Component | Role |
|-----------|------|
| **ANN-MB1 Antenna** | Multi-band GNSS antenna |
| **ZED-F9P** | RTK GNSS receiver — computes cm-level position using RTCM corrections |
| **OpenLog Artemis (OLA)** | Data logger — reads ZED-F9P via I2C, samples built-in IMU, writes to SD |
| **ICM-20948 IMU** | Built into OLA — 9-DoF accelerometer/gyroscope/magnetometer at 10 Hz |
| **ESP32** | NTRIP client — fetches RTK corrections from Polaris and forwards to ZED-F9P via UART |
| **Polaris Network RTK** | Cloud NTRIP caster — streams RTCM3 correction messages |
| **MicroSD Card** | Stores raw UBX GPS data (`dataLog.ubx`) and IMU CSV (`imuLog.csv`) |
