# Credit : Mohammed Noureldin
# https://www.udemy.com/course/usb-behind-the-scenes-hands-on-hid-firmware-development/

# Installs Wireshark and its dependencies.

sudo apt install wireshark

# Ensures that users without root permissions can capture packets.

sudo dpkg-reconfigure wireshark-common

# Adds the current user to wireshark group.

sudo adduser $USER wireshark

## You need to execute the following two commands every time you reboot your Linux system:

# Loads usbmon module to the kernel.

modprobe usbmon

# Gives regular users permissions to use the kernel module.

sudo setfacl -m u:$USER:r /dev/usbmon*

--------

To monitor the system log in Linux, simply execute the following command in Linux command line:

sudo tail -f /var/log/syslog
