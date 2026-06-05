import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

fig, ax = plt.subplots(figsize=(14, 9))
ax.set_xlim(0, 14)
ax.set_ylim(0, 9)
ax.axis('off')
fig.patch.set_facecolor('#f8f9fa')

C_GNSS    = '#1565c0'
C_MCU     = '#6a1b9a'
C_STORAGE = '#4e342e'
C_CLOUD   = '#bf360c'
C_BG_BUOY = '#e3f2fd'
C_BG_NET  = '#fff8e1'

# Group background boxes
ax.add_patch(FancyBboxPatch((0.3, 0.4), 10.2, 8.2,
    boxstyle="round,pad=0.2", linewidth=2,
    edgecolor='steelblue', facecolor=C_BG_BUOY, zorder=0, alpha=0.5))
ax.text(5.4, 8.45, 'Buoy Hardware', ha='center', fontsize=13,
        fontweight='bold', color='steelblue')

ax.add_patch(FancyBboxPatch((10.8, 4.2), 2.9, 3.1,
    boxstyle="round,pad=0.2", linewidth=2,
    edgecolor='darkorange', facecolor=C_BG_NET, zorder=0, alpha=0.5))
ax.text(12.25, 7.15, 'Internet', ha='center', fontsize=12,
        fontweight='bold', color='darkorange')


def draw_block(cx, cy, w, h, lines, color, fontsize=9.5):
    ax.add_patch(FancyBboxPatch(
        (cx - w/2, cy - h/2), w, h,
        boxstyle="round,pad=0.2", linewidth=2,
        edgecolor=color, facecolor='white', zorder=2))
    ax.text(cx, cy, '\n'.join(lines), ha='center', va='center',
            fontsize=fontsize, color=color, fontweight='bold',
            multialignment='center', zorder=3, linespacing=1.5)


def draw_arrow(x1, y1, x2, y2, label='', color='gray',
               bidir=False, style='arc3,rad=0'):
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(
            arrowstyle=('<->' if bidir else '->'),
            color=color, lw=1.8,
            connectionstyle=style),
        zorder=1)
    if label:
        mx, my = (x1 + x2) / 2, (y1 + y2) / 2
        ax.text(mx, my, label, ha='center', va='center',
                fontsize=7.5, color=color, multialignment='center',
                linespacing=1.4,
                bbox=dict(boxstyle='round,pad=0.25', fc='white',
                          ec='none', alpha=0.9))


# Component centers
ant_pos = (3.0, 7.0)
zed_pos = (3.0, 4.5)
ola_pos = (7.5, 4.5)
esp_pos = (7.5, 7.0)
sd_pos  = (7.5, 2.0)
pol_pos = (12.25, 5.75)

W = 3.2
H_sm, H_md, H_lg = 0.9, 1.1, 1.4

draw_block(*ant_pos, W, H_sm,
    ['ANN-MB1', 'GNSS Antenna'], C_GNSS)

draw_block(*zed_pos, W, H_lg,
    ['ZED-F9P', 'RTK Receiver'], C_GNSS)

draw_block(*ola_pos, W, H_lg,
    ['OpenLog Artemis', '(Apollo3 + ICM-20948', 'IMU + Data Logger)'], C_MCU, fontsize=8.5)

draw_block(*esp_pos, W, H_sm,
    ['ESP32', 'NTRIP Client'], C_MCU)

draw_block(*sd_pos, W, H_md,
    ['MicroSD Card', 'dataLog.ubx  ·  imuLog.csv'], C_STORAGE, fontsize=8.5)

draw_block(*pol_pos, 2.6, H_lg,
    ['Polaris Network RTK', 'polaris.pointonenav.com'], C_CLOUD, fontsize=8.5)

# Antenna → ZED-F9P (straight down, RF)
draw_arrow(ant_pos[0], ant_pos[1] - H_sm/2,
           zed_pos[0], zed_pos[1] + H_lg/2,
           'RF signal', C_GNSS)

# ZED-F9P ↔ OLA (straight right, I2C)
draw_arrow(zed_pos[0] + W/2, zed_pos[1],
           ola_pos[0] - W/2, ola_pos[1],
           'I2C / Qwiic', 'gray', bidir=True)

# ESP32 → ZED-F9P (curved, UART RTCM)
draw_arrow(esp_pos[0], esp_pos[1] - H_sm/2,
           zed_pos[0] + W/2, zed_pos[1],
           'UART 115200 baud\nRTCM3 corrections', C_MCU,
           style='arc3,rad=0.25')

# ESP32 ↔ Polaris (straight right, WiFi/LTE)
draw_arrow(esp_pos[0] + W/2, esp_pos[1],
           pol_pos[0] - 2.6/2, pol_pos[1],
           'WiFi / LTE\n(NTRIP)', C_CLOUD, bidir=True)

# OLA → MicroSD (straight down, SPI)
draw_arrow(ola_pos[0], ola_pos[1] - H_lg/2,
           sd_pos[0], sd_pos[1] + H_md/2,
           'SPI', C_MCU)

ax.set_title('RTK Wave Buoy — System Block Diagram',
             fontsize=15, fontweight='bold', pad=12, color='#212121')

plt.tight_layout(pad=0.5)
out = 'system_block_diagram.png'
plt.savefig(out, dpi=150, bbox_inches='tight', facecolor=fig.get_facecolor())
print(f"Saved → {out}")
