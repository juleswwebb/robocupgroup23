#ifndef MAGNET_CONTROL_H_
#define MAGNET_CONTROL_H_

// Controls the driver's logic input only; the magnet coil needs its own supply.
void magnet_control_init();
void magnet_control_set(bool enabled);
void magnet_control_off();
void magnet_control_update();
bool magnet_control_is_on();

#endif /* MAGNET_CONTROL_H_ */
