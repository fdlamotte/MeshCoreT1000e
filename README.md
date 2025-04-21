# MeshCoreT1000e

Custom MeshCore support for Seeed T1000e tracker 

This repo contains the source files and the PlatformIO environments to build custom firmwares using [MeshCore](https://meshcore.co.uk) on the [Seeed T1000e Tracker](https://www.seeedstudio.com/SenseCAP-Card-Tracker-T1000-E-for-Meshtastic-p-5913.html).

## How to test

On [MeshCore web flasher](https://flasher.meshcore.co.uk/) you'll find official firmware for t1000e. This repo adds some functionalities that have not been added yet into main repo because there we try to find more generic ways of doing things.

I provide some uf2s of the custom firmware in this repo's relase. To flash the `.uf2`, put your device in DFU by pressing the button and plugging the cable twice and copy the firmware file to the removable drive.

## Features

* gps support (3s press to enable)
* some power optimization
* BLE advertisement stops after 5min (short press on the button re-enables it for 5min)
* Notification led (upstreamed)
* Power button support (long press > 5s) (upstreamed)

upcoming
* Buzzer
* Serial console for options specific to t1000e

## Buttons and leds 

### Notification led

Leds shows the status of the device, it varies in brightness (pwm) and time. There are two time at the moment, communication status and gps status.

Communication status (mostly on state) :
* Unread messages : 95% brightness, 200ms
* BLE advertisement active : 20% brightness, 2s
* BLE advertisement inactive : 20% brightness, 3s

GPS status :
* GPS off : led off, 3s
* GPS on, no fix : led off, 2s
* GPS on, fixed : 40% brightness, 2s (so when there is a fix, led brightness appears constant)

### User button

* long press (>4s) : turns device off (led stops)
* medium press (>1s) : toggles gps
* short press : activates BLE adevertising for 5min

### Cli

This firmware features a cli, accessible through serial port or with meshcli (using `meshcli cli cmd` or `meshcli @cmd`)

Commands are the following :
* `advert`: send avert
* `floodadv`: send flood advert
* `uptime`: print uptime
* `bat`: give bat status (percent and voltage)
* `reboot`: reboots the device
* `pinval`: reads a pin of the MCU
* `gps_sync`: sync time at next gps fix
* `get/set` : change a toggle/parameter
* `saveprefs` : write prefs to flash (usefull for storing gps position)

Parameters are :
* `blesleep`: time before ble advertisement stops
* `ble_tx`: bluetooth tx power
* `pin`: bluetooth pairing pin
* `gps`: toggle gps on/off
* `repeat`: toggle repeat (not saved, off by default)
* `rx_boost`: set rx_boost, can be `on`, `off` or `auto` where `rx_boost` is active when ble is active ...

## To compile the firmware

The firmware are build using PlatformIO on VSCode.

This repository only comes with support code for the T1000e, the MeshCore files come from the [main MeshCore repository](https://github.com/ripplebiz/MeshCore) managed by Scott Powell and should be provided apart. 

To setup the build environment, you should clone both repositories at the same level and add them to a VSCode workspace, build targets for the T1000e should then be available. You can build and run (if the T1000e is connected the code should automatically be uploaded to the device).

 <pre>$ cd &lt;meshcode_workspace_dir&gt;

$ git clone https://github.com/ripplebiz/MeshCore
Cloning into 'MeshCore'...
remote: Enumerating objects: 1080, done.
remote: Counting objects: 100% (236/236), done.
remote: Compressing objects: 100% (84/84), done.
remote: Total 1080 (delta 201), reused 167 (delta 144), pack-reused 844 (from 1)
Receiving objects: 100% (1080/1080), 282.46 KiB | 2.62 MiB/s, done.
Resolving deltas: 100% (655/655), done.

$ git clone https://github.com/fdlamotte/MeshCoreT1000e
Cloning into 'MeshCoreT1000e'...
remote: Enumerating objects: 150, done.
remote: Counting objects: 100% (150/150), done.
remote: Compressing objects: 100% (92/92), done.
remote: Total 150 (delta 58), reused 125 (delta 37), pack-reused 0 (from 0)
Receiving objects: 100% (150/150), 41.04 KiB | 808.00 KiB/s, done.
Resolving deltas: 100% (58/58), done.

$ ls
MeshCore  MeshCoreT1000e</pre>

Once the environment is setup, you can customize the firmware by adding your code and writing your own build targets in `platformio.ini`.