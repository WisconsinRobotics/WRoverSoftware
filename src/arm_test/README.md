You have to export this:

echo "/usr/lib/phoenix6" | sudo tee /etc/ld.so.conf.d/phoenix6.conf
sudo ldconfig

to make the code actually see the library. I added it to the bashrc script in the basestation but will need to do the same in the JETSON. 
