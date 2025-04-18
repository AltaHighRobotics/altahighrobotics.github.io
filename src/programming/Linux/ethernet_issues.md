
---
title: "Ethernet issues"
parent: Linux
---
If the ethernet device shows as "unavailable" when using this command:

```bash
nmcli device status
```

You need to restart the network manager service, by doing:
```bash
sudo systemctl restart NetworkManager
```
This will temporarily disconnect you from all network devices but will connect again with (hopefully) all devices working again



#commands/nmcli #commands/systemctl
