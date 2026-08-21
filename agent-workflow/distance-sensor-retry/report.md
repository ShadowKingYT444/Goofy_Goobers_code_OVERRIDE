Changed files:
distance_working/src/main.cpp
distance_working/include/main.h
tools/distance_server.py

Commands run:
PROS build/upload to slot 1 on COM7.
Direct pyserial reads on COM6 and COM7.
PROS screen captures on COM7.
Python syntax check for tools/distance_server.py.
HTTP /data check against http://127.0.0.1:8771/data.

Evidence:
COM6 remained silent.
Brain screen capture proved the distance program loop was running.
Barcode decode returned mm=1, confidence=10, sample=19.
/data returned connected=true with samples and message "Reading Brain screen barcode on COM7; USB console is silent."

Remaining risks:
Screen-capture fallback is slower than real serial and depends on COM7 being free.
Current distance value is 1 mm with confidence 10, which likely means the sensor is pointed too close, blocked, or not seeing a good target.
