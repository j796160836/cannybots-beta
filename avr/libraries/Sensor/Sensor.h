#ifndef Sensor_h
#define Sensor_h

void autoSample(int channel, long rate, void (*f)(int));
void noAutoSample();

int vcc();

#endif