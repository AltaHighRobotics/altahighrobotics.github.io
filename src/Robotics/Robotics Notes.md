---
title: Robotics Notes 
parent: Robotics
---
___

## RoboRIO images
Download 2025 roborio image: [external link](https://pizza2d1.duckdns.org/files/WorkingRoboRIO.img.gz) [unblocked external link](http:136.60.227.41/files/WorkingRoboRIO.img.gz)
##### Linx:
###### Create a compressed image file with a working image file

```bash
dd if=/dev/sdb | gzip > backup.img.gz
```

###### Flash a drive with compressed image file

```bash
cat backup.img.gz | gunzip | dd of=/dev/sdb
```


Example code for our purposes: 
###### This will echo the contents of the RoboRio image file, decompress it with gunzip, and then overwrite existing data on the /dev/mmcblk0p1 storage device, status and bs parameters add verbosity and speed
```bash
sudo cat WorkingRoboRIO.img.gz status=progress bs=32M| gunzip | dd of=/dev/mmcblk0p1 status=progress bs=32M
# Make sure you are outputting to the right drive and not your own, that will brick your laptop
```


## Virtual Environment
Get into a virtual environment:
##### Windows:
Run in powershell:
```batch
Set-ExecutionPolicy -ExecutionPolicy Unrestricted -Scope CurrentUser
```

Run in repository destination:
```
venv env

# Or if your python is weird:
python3 -m venv env
```
## NOT FINISHED

##### Linux:
Make sure that both `python3.12-venv` and `python3-pip` are installed beforehand

In the current directory
```bash
# Create virtual environment
python3 -m venv .env

# Enter virtual environment
source .env/bin/activate
```

Once you are in the virtual environment you are able to download any packages that you want and run the modules without any issue, BUT once you leave the environment (which can be seen in terminal as no longer having `(.env)` at the terminal entry line)

To get back into the virtual environment, you just run the same source command:
```bash
source .env/bin/activate
```
Which you may have to do each time you want to use that specific environment
