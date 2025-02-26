echo  'KERNEL=="tty*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE:="777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
service udev reload
sleep 2
service udev restart


