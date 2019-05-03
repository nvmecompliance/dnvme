This is the dnvm repository in the Nvme Compliance project.

This repo contains the Linux driver for the NVMe Compliance Suite.

Contact compliance@nvmexpress.org for more information.

## Compilation

To compile:
```
# don't use sudo
make
```

## Installation

To install
```
sudo make install
```

## Running

To run after install (unload the kernel nvme driver first)
```
sudo modprobe -r nvme # unload kernel driver

sudo modprobe dnvme
```

## Potential Issues

If you run into issues, check dmesg.

If you see something like:
```
DMAR: DRHD: handling fault status reg 3
```
try setting (in your grub config (for ex: /etc/default/grub))
```
intel_iommu=off
```
then run
```
sudo update-grub
```

