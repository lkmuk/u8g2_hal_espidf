## Expected results

### For a single display

Initially, you see all pixels on. 
Soon after, you see a 6-digit number representing a counter.
The counter is incremented by 1 at (approximately) 10 Hz from 0 to 999999.
When exceeeding 999999 (so roughly after 99999 seconds), 
the counter value is reset to 0.
You should also see the right and bottom edge of the screen remaining 
on all the time.

### For two display sharing the same 

(You should uncomment line 25, to define SECOND_OLED_IS_AVAILABLE)
You see the first display doing the same thing while the second display
only shows the 3 least significant digits (so updated at the same rate).



## Steps for reproducing the results

Assuming you already have installed ESP-IDF.
Activate the environment. 
Make this directory the active working directory

Download U8G2 to somewhere.
Let ESP-IDF find it by setting `EXTRA_COMPONENT_DIRS` in the project-level CMakeList, 
or specify it in idf_component.yml.
For quick start, this is already done for you in the idf_component.yml.


```bash
idf.py set-target esp32c6  # replace it with your own^
idf.py menuconfig # optional
idf.py -p /dev/ttyACM0 build flash monitor
```

^ You are expected to also change the code, especially which I2C port.



