# I try to write a linux device driver on RPI3.....
   usage: 
         step 1:  build the linux driver
                  make all
         
         step 2:
              sudo insmod ./pwm.ko gpio_pin=2
           or sudo insmod ./pwm.ko
           
         step 3:  build the device file
                 sudo mknod /dev/pwm c 80 0
         step4 :
                read file
                 cat /dev/pwm  
              
           
