___
## RoboRIO images

###### Create a compressed image file with a working image file

```bash
dd if=/dev/sdb | gzip > backup.img.gz
```

###### Flash a drive with compressed image file

```bash
cat backup.img.gz | gunzip | dd of=/dev/sdb
```


Example code for our purposes: 
```bash
sudo dd if=/dev/sda status=progress bs=32M| gzip > arch_hyprland.img.gz
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



#commands/source #commands/dd