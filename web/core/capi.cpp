#include "drone_core.hpp"
#ifdef __EMSCRIPTEN__
  #include <emscripten/emscripten.h>
  #define KA EMSCRIPTEN_KEEPALIVE
#else
  #define KA
#endif
using namespace dronesim;

extern "C" {
KA DroneCore* skysim_create()                         { return new DroneCore(); }
KA void skysim_destroy(DroneCore* d)                  { delete d; }
KA void skysim_reset(DroneCore* d,double x,double y,double z){ d->reset(x,y,z); }
KA void skysim_arm(DroneCore* d,int on)               { if(on) d->arm(); else d->disarm(); }
KA void skysim_set_attitude(DroneCore* d,double r,double p,double yr,double thr){ d->set_attitude_setpoint(r,p,yr,thr); }
KA void skysim_set_motors(DroneCore* d,double* t,int n){ d->set_motors(t,n); }
KA void skysim_set_wind(DroneCore* d,double x,double y,double z){ d->set_wind(x,y,z); }
KA void skysim_step(DroneCore* d,double dt)           { d->step(dt); }
KA void skysim_get_obs(DroneCore* d,double* out12)    { d->get_obs(out12); }
KA double skysim_altitude(DroneCore* d){ return d->telem().altitude; }
KA double skysim_vspeed(DroneCore* d)  { return d->telem().vertical_speed; }
KA double skysim_thrust(DroneCore* d)  { return d->telem().total_thrust; }
KA double skysim_roll(DroneCore* d)    { return d->telem().roll_deg; }
KA double skysim_pitch(DroneCore* d)   { return d->telem().pitch_deg; }
KA double skysim_yaw(DroneCore* d)     { return d->telem().yaw_deg; }
KA double skysim_power(DroneCore* d)         { return d->telem().power_draw; }
KA double skysim_battery_voltage(DroneCore* d){ return d->telem().battery_voltage; }
KA double skysim_battery_soc(DroneCore* d)   { return d->telem().battery_soc; }
KA double skysim_battery_current(DroneCore* d){ return d->telem().battery_current; }
KA int    skysim_battery_cutoff(DroneCore* d){ return d->telem().battery_cutoff ? 1 : 0; }
KA void   skysim_set_battery_thrust_coupling(DroneCore* d,int on){ d->set_battery_thrust_coupling(on!=0); }
}
