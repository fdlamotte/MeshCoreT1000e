# MeshCoreT1000e
MeshCore support for Seeed T1000e tracker 

This repo contains the source files and the PlatformIO environments to build firmwares using [MeshCore](https://meshcore.co.uk) on the [Seeed T1000e Tracker](https://www.seeedstudio.com/SenseCAP-Card-Tracker-T1000-E-for-Meshtastic-p-5913.html).

## How to test

You'll find firmwares baked by Andy Kirby on the [MeshCore web flasher](https://flasher.meshcore.co.uk/).

Download the `.uf2`, put your device in DFU by pressing the button and plugging the cable twice and copy the firmware file to the removable drive.

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