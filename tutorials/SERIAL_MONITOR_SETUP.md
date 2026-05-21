# Serial Monitor Setup

Connect to the OpenLog Artemis (OLA) over serial to view boot messages, IMU data, and access the menu. This guide covers two free serial terminal applications — one for each platform.

| Platform | Application | Download |
|----------|-------------|----------|
| Windows | TeraTerm | https://github.com/TeraTermProject/teraterm/releases |
| macOS | CoolTerm | https://freeware.the-meiers.org |

---

## Windows — TeraTerm

### Install

1. Go to https://github.com/TeraTermProject/teraterm/releases
2. Download the latest `.exe` installer (e.g. `teraterm-5.x.exe`)
3. Run the installer and follow the prompts — default options are fine

### Connect to the OLA

1. Plug the OLA into your computer via USB-C (or USB-serial adapter)
2. Open TeraTerm — a **New Connection** dialog will appear automatically
3. Select **Serial** and choose the correct COM port from the dropdown
   - If you are unsure which port, check **Device Manager → Ports (COM & LPT)**
   - It will appear as something like `USB Serial Port (COM3)`
4. Click **OK**
5. Go to **Setup → Serial Port**
6. Set the following:
   - **Baud rate:** 115200
   - **Data:** 8 bit
   - **Parity:** None
   - **Stop bits:** 1
   - **Flow control:** None
7. Click **OK**
8. Press the **reset button** on the OLA — you should see boot messages appear

### What You Should See

```
Artemis OpenLog GNSS v3.2
IMU online
SD card online
Data logging online
GNSS offline
2026/04/07 15:36:55.00,IMU,510.74,-133.30,-860.35,...
```

### Accessing the Menu

Type any character in the TeraTerm window and press **Enter** — the OLA menu will appear. To return to logging press **x**.

### Save Settings for Next Time

Go to **Setup → Save Setup** — TeraTerm will remember your port and baud rate.

---

## macOS — CoolTerm

### Install

1. Go to https://freeware.the-meiers.org
2. Scroll down to **CoolTerm** and click the macOS download link
3. Unzip the downloaded file and drag **CoolTerm.app** to your Applications folder
4. On first launch, macOS may warn about an unverified developer — go to **System Settings → Privacy & Security** and click **Open Anyway**

### Connect to the OLA

1. Plug the OLA into your Mac via USB-C (or USB-serial adapter)
2. Open CoolTerm
3. Click **Options**
4. Under **Serial Port**:
   - **Port:** Select `cu.usbserial-XXXXX` from the dropdown
     - Always use `cu.*` — not `tty.*` (the tty variant will hang)
     - Click **Re-Scan Serial Ports** if your port does not appear
   - **Baudrate:** 115200
   - **Data Bits:** 8
   - **Parity:** None
   - **Stop Bits:** 1
5. Click **OK**
6. Click **Connect**
7. Press the **reset button** on the OLA — you should see boot messages appear

### What You Should See

```
Artemis OpenLog GNSS v3.2
IMU online
SD card online
Data logging online
GNSS offline
2026/04/07 15:36:55.00,IMU,510.74,-133.30,-860.35,...
```

### Accessing the Menu

Click in the CoolTerm window and type any character — the OLA menu will appear. To return to logging press **x**.

### Save Settings for Next Time

Go to **File → Save** to save a CoolTerm connection file. Double-clicking it next time will open CoolTerm with your settings pre-loaded.

---

## Troubleshooting

| Problem | Fix |
|---------|-----|
| No ports visible (Windows) | Install the USB-serial driver. For CP210x adapters: search "CP210x driver Silicon Labs". For CH340: search "CH340 driver". |
| No ports visible (macOS) | Same driver issue. For CP210x: Silicon Labs driver. For CH340: search "CH340 Mac driver". |
| Port visible but no data | Check baud rate is 115200. Try pressing reset on the OLA. |
| Garbled text | Baud rate mismatch — make sure it is set to exactly 115200. |
| macOS: port hangs / no data | Make sure you selected `cu.*` not `tty.*` in CoolTerm. |
| "Access denied" (Windows) | Another application (e.g. Arduino IDE) is using the port — close it first. |
| OLA boots but stops outputting | It may have entered low-power mode. Press reset to restart. |
